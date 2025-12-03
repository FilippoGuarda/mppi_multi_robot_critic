#include "mppi_multi_robot_critic/interaction_aware_critic.hpp"

#include <cmath>
#include <algorithm>
#include <pluginlib/class_list_macros.hpp>

namespace nav2_interaction_critic
{

void InteractionAwareCritic::initialize()
{
  // Get parameter getter helper
  auto get_param = parameters_handler_->getParamGetter(name_);

  // Load cost weights
  get_param(weights_.static_obstacle_weight, "static_obstacle_weight", 100.0f);
  get_param(weights_.rotation_weight, "rotation_weight", 5.0f);
  get_param(weights_.tracking_weight, "tracking_weight", 10.0f);
  get_param(weights_.speed_weight, "speed_weight", 50.0f);
  get_param(weights_.dynamic_collision_weight, "dynamic_collision_weight", 100.0f);
  get_param(weights_.regulation_weight, "regulation_weight", 50.0f);

  // Load configuration parameters
  get_param(collision_penalty_, "collision_penalty", 1000.0f);
  get_param(rotation_penalty_slow_, "rotation_penalty_slow", 1.0f);
  get_param(rotation_penalty_fast_, "rotation_penalty_fast", 2.0f);
  get_param(speed_threshold_, "speed_threshold", 0.5f);
  get_param(max_speed_, "max_speed", 1.7f);
  get_param(angular_margin_, "angular_margin", 0.2f);
  get_param(right_of_way_radius_, "right_of_way_radius", 2.0f);
  get_param(min_agent_velocity_, "min_agent_velocity", 0.5f);

  // Load tracked agent names
  get_param(tracked_agent_names_, "tracked_agents", std::vector<std::string>());

  // Initialize TF buffer
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(parent_.lock()->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(logger_, "InteractionAwareCritic initialized with %zu tracked agents",
    tracked_agent_names_.size());
}

void InteractionAwareCritic::score(mppi::CriticData & data)
{
  if (!enabled_) {
    return;
  }

  // Get global frame from costmap
  std::string global_frame = costmap_ros_->getGlobalFrameID();

  // Update other agents' states
  updateOtherAgentsState(global_frame);

  // Extract local goal for ego agent
  Eigen::Vector2f ego_goal = extractLocalGoal(data.path);

  // MPPI Humble API: trajectories is a Trajectories struct with x, y members only
  // Each is an xtensor<float, 2> with shape [batch_size, time_steps]
  const auto & trajectories = data.trajectories;
  size_t batch_size = trajectories.x.shape(0);

  // Score each trajectory in the batch
  for (size_t b = 0; b < batch_size; ++b)
  {
    float total_cost = 0.0f;

    // Extract single batch trajectory views
    auto x_traj = xt::view(trajectories.x, b, xt::all());
    auto y_traj = xt::view(trajectories.y, b, xt::all());

    // 1. Agent-centric costs (tracking, rotation, collisions with static obstacles)
    total_cost += computeTrackingCost(x_traj, y_traj, ego_goal) * 
      weights_.tracking_weight;
    total_cost += computeRotationCost(x_traj, y_traj) * weights_.rotation_weight;
    total_cost += computeStaticCollisionCost(x_traj, y_traj) * 
      weights_.static_obstacle_weight;

    // 2. Configuration costs (multi-agent interactions)
    for (const auto & [agent_name, other_agent] : other_agents_)
    {
      // Dynamic collision cost
      Eigen::Vector2f start_pos(x_traj(0), y_traj(0));
      if (detectDynamicCollision(start_pos, other_agent))
      {
        total_cost += collision_penalty_ * weights_.dynamic_collision_weight;
      }

      // Regulation violation cost
      total_cost += computeRegulationCost(x_traj, y_traj, other_agent) *
        weights_.regulation_weight;
    }

    data.costs(b) += total_cost;
  }
}

void InteractionAwareCritic::updateOtherAgentsState(const std::string & frame_id)
{
  other_agents_.clear();

  for (const auto & agent_name : tracked_agent_names_)
  {
    try
    {
      // Lookup transform for other agent
      geometry_msgs::msg::TransformStamped tf =
        tf_buffer_->lookupTransform(frame_id, agent_name, tf2::TimePointZero);

      AgentState agent;
      agent.position.x() = static_cast<float>(tf.transform.translation.x);
      agent.position.y() = static_cast<float>(tf.transform.translation.y);

      // Extract heading from quaternion
      double yaw = tf2::getYaw(tf.transform.rotation);
      agent.heading = static_cast<float>(yaw);

      // Estimate velocity using constant velocity model
      // In production, subscribe to /agent_name/odom or /agent_name/cmd_vel
      agent.velocity.x() = std::cos(agent.heading) * 0.5f;  // Default 0.5 m/s
      agent.velocity.y() = std::sin(agent.heading) * 0.5f;

      other_agents_[agent_name] = agent;

      RCLCPP_DEBUG(logger_, "Updated agent %s at (%.2f, %.2f)",
        agent_name.c_str(), agent.position.x(), agent.position.y());
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN_THROTTLE(logger_, *parent_.lock()->get_clock(), 2000,
        "Could not get transform for agent %s: %s", agent_name.c_str(), ex.what());
    }
  }
}

Eigen::Vector2f InteractionAwareCritic::predictLocalGoal(
  const AgentState & agent, float horizon)
{
  // Constant velocity model: p_goal = k_s * T * v + p
  float k_s = 1.5f;
  Eigen::Vector2f predicted = agent.position + (agent.velocity * horizon * k_s);

  // Check if predicted goal is in collision; if so, return agent position
  unsigned int mx, my;
  if (!costmap_->worldToMap(predicted.x(), predicted.y(), mx, my))
  {
    return agent.position;  // Out of bounds
  }

  if (costmap_->getCost(mx, my) >= lethal_cost_)
  {
    return agent.position;  // In collision
  }

  return predicted;
}

float InteractionAwareCritic::computeTrackingCost(
  const xt::xtensor<float, 1> & x_traj,
  const xt::xtensor<float, 1> & y_traj,
  const Eigen::Vector2f & goal)
{
  float cost = 0.0f;
  size_t steps = x_traj.shape(0);

  if (steps == 0) {
    return cost;
  }

  // Calculate initial distance for normalization
  float initial_dx = x_traj(0) - goal.x();
  float initial_dy = y_traj(0) - goal.y();
  float initial_dist = std::sqrt(initial_dx * initial_dx + initial_dy * initial_dy);

  if (initial_dist < 0.01f) {
    return 0.0f;  // Already at goal
  }

  // Sum normalized distances along trajectory
  for (size_t t = 0; t < steps; ++t)
  {
    float dx = x_traj(t) - goal.x();
    float dy = y_traj(t) - goal.y();
    float dist = std::sqrt(dx * dx + dy * dy);
    cost += dist / initial_dist;
  }

  return cost / steps;
}

float InteractionAwareCritic::computeRotationCost(
  const xt::xtensor<float, 1> & x_traj,
  const xt::xtensor<float, 1> & y_traj)
{
  float cost = 0.0f;
  size_t steps = x_traj.shape(0);

  if (steps < 2) {
    return cost;
  }

  // Estimate rotation from position changes
  // Compute heading at each step and penalize changes
  for (size_t t = 1; t < steps; ++t)
  {
    // Heading change from trajectory
    float dx_prev = x_traj(t - 1) - (t > 1 ? x_traj(t - 2) : x_traj(0));
    float dy_prev = y_traj(t - 1) - (t > 1 ? y_traj(t - 2) : y_traj(0));
    
    float dx_curr = x_traj(t) - x_traj(t - 1);
    float dy_curr = y_traj(t) - y_traj(t - 1);

    float heading_prev = std::atan2(dy_prev, dx_prev);
    float heading_curr = std::atan2(dy_curr, dx_curr);

    float dheading = heading_curr - heading_prev;
    // Normalize to [-pi, pi]
    while (dheading > M_PI) {
      dheading -= 2.0f * M_PI;
    }
    while (dheading < -M_PI) {
      dheading += 2.0f * M_PI;
    }

    cost += std::abs(dheading);
  }

  return cost / steps;
}

float InteractionAwareCritic::computeStaticCollisionCost(
  const xt::xtensor<float, 1> & x_traj,
  const xt::xtensor<float, 1> & y_traj)
{
  float cost = 0.0f;
  size_t steps = x_traj.shape(0);

  for (size_t t = 0; t < steps; ++t)
  {
    unsigned int mx, my;
    if (!costmap_->worldToMap(x_traj(t), y_traj(t), mx, my))
    {
      cost += collision_penalty_;
      continue;
    }

    unsigned char cell_cost = costmap_->getCost(mx, my);
    if (cell_cost >= lethal_cost_)
    {
      cost += collision_penalty_;
    }
    else if (cell_cost > nav2_costmap_2d::FREE_SPACE)
    {
      // Graduated penalty for inscribed obstacle costs
      cost += (static_cast<float>(cell_cost) / 255.0f) * collision_penalty_ * 0.5f;
    }
  }

  return cost / steps;
}

float InteractionAwareCritic::computeRegulationCost(
  const xt::xtensor<float, 1> & x_traj,
  const xt::xtensor<float, 1> & y_traj,
  const AgentState & other_agent)
{
  float cost = 0.0f;
  size_t steps = x_traj.shape(0);

  if (steps < 2) {
    return cost;
  }

  // Check regulation violations along trajectory
  for (size_t t = 1; t < steps; ++t)
  {
    Eigen::Vector2f robot_pos(x_traj(t), y_traj(t));

    // Estimate robot velocity from trajectory (position difference)
    Eigen::Vector2f robot_vel(
      x_traj(t) - x_traj(t - 1),
      y_traj(t) - y_traj(t - 1));

    // Check right-of-way violation
    if (isRightOfWayViolation(robot_pos, robot_vel, other_agent))
    {
      cost += collision_penalty_;
    }

    // Check head-on encounter
    if (isHeadOnEncounter(robot_vel, other_agent.velocity))
    {
      cost += collision_penalty_ * 0.5f;
    }
  }

  return cost / steps;
}

bool InteractionAwareCritic::isRightOfWayViolation(
  const Eigen::Vector2f & robot_pos,
  const Eigen::Vector2f & robot_vel,
  const AgentState & other_agent)
{
  Eigen::Vector2f relative_pos = other_agent.position - robot_pos;
  float distance = relative_pos.norm();

  // Check if other agent is within right-of-way radius
  if (distance > right_of_way_radius_)
  {
    return false;
  }

  // Check if other agent is on starboard (right) side using cross product
  // cross = v_x * rel_y - v_y * rel_x
  // positive = left, negative = right
  float cross = robot_vel.x() * relative_pos.y() - robot_vel.y() * relative_pos.x();

  if (cross >= 0.0f)
  {
    return false;  // Agent is on port (left) side
  }

  // Check if other agent is moving (not stationary)
  if (other_agent.velocity.norm() < min_agent_velocity_)
  {
    return false;
  }

  // Check relative velocity condition (Eq. 13 from paper)
  float cross_mag = std::abs(
    robot_vel.x() * other_agent.velocity.y() -
    robot_vel.y() * other_agent.velocity.x());

  float threshold = robot_vel.norm() * other_agent.velocity.norm() *
    std::sin(-M_PI / 2.0f + angular_margin_);

  return cross_mag < threshold;
}

bool InteractionAwareCritic::isHeadOnEncounter(
  const Eigen::Vector2f & robot_vel,
  const Eigen::Vector2f & other_vel)
{
  // Check if velocities are approximately opposite (head-on)
  if (robot_vel.norm() < min_agent_velocity_ ||
    other_vel.norm() < min_agent_velocity_)
  {
    return false;
  }

  // Dot product test: head-on means dot product close to -1
  float dot_product = robot_vel.dot(other_vel);
  float vel_product = robot_vel.norm() * other_vel.norm();

  // If dot product is very negative (opposite directions)
  return dot_product < (-vel_product * std::cos(angular_margin_));
}

bool InteractionAwareCritic::detectDynamicCollision(
  const Eigen::Vector2f & robot_pos,
  const AgentState & other_agent)
{
  float distance = (robot_pos - other_agent.position).norm();
  float min_distance = other_agent.radius + 0.3f;  // Robot radius ~ 0.3m

  return distance < min_distance;
}

Eigen::Vector2f InteractionAwareCritic::extractLocalGoal(const mppi::models::Path & path)
{
  // Use last point of path as local goal
  if (path.x.shape(0) > 0)
  {
    size_t last_idx = path.x.shape(0) - 1;
    return Eigen::Vector2f(path.x(last_idx), path.y(last_idx));
  }

  // Fallback: return origin if path is empty
  return Eigen::Vector2f(0.0f, 0.0f);
}

}  // namespace nav2_interaction_critic

// Plugin registration with correct base class
PLUGINLIB_EXPORT_CLASS(
  nav2_interaction_critic::InteractionAwareCritic,
  mppi::critics::CriticFunction)