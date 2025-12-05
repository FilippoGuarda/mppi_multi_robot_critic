#include "mppi_multi_robot_critic/multi_agent_critic.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

void MultiAgentInteractionCritic::initialize()
{
  RCLCPP_INFO(logger_, "Initializing MultiAgentInteractionCritic");
  
  auto node = parent_.lock();
  if (!node) {
    throw std::runtime_error("Parent node is invalid");
  }
  node_ = node;

  // Extract robot namespace from node name
  robot_namespace_ = node_->get_namespace();
  if (robot_namespace_ == "/") {
    robot_namespace_ = "";
  }

  // Load parameters using getParamGetter
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(collision_penalty_, "collision_penalty", 1000000.0);
  getParam(collision_margin_, "collision_margin", 0.1);
  getParam(use_all_trajectory_points_, "use_all_trajectory_points", false);
  getParam(estimate_velocity_, "estimate_velocity", true);
  getParam(max_agent_distance_, "max_agent_distance", 10.0);
  getParam(velocity_timeout_, "velocity_timeout", 1.0);
  getParam(enabled_, "enabled", true);

  RCLCPP_INFO(
    logger_,
    "MultiAgentInteractionCritic configured:\n"
    "  collision_penalty: %.0f\n"
    "  collision_margin: %.3f m\n"
    "  use_all_trajectory_points: %s\n"
    "  estimate_velocity: %s\n"
    "  max_agent_distance: %.1f m",
    collision_penalty_,
    collision_margin_,
    use_all_trajectory_points_ ? "true" : "false",
    estimate_velocity_ ? "true" : "false",
    max_agent_distance_);
}

void MultiAgentInteractionCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  // Prune predictions older than timeout
  prune_stale_predictions();

  // Get ego robot radius for collision checking
  double ego_radius = get_ego_radius();

  // Lock access to other agents during scoring
  std::lock_guard<std::mutex> lock(agents_mutex_);
  
  if (other_agents_.empty()) {
    return;  // No other agents to avoid
  }

  // Access trajectories from CriticData
  // data.trajectories has shape: [batch_size, time_steps, state_dim]
  auto & trajectories = data.trajectories;
  
  // Iterate over all trajectory samples in the batch
  for (size_t i = 0; i < data.costs.rows(); ++i) {
    // Extract trajectory poses for this sample
    std::vector<geometry_msgs::msg::PoseStamped> ego_trajectory;
    ego_trajectory.reserve(trajectories.x.cols());
    
    for (size_t t = 0; t < trajectories.x.cols(); ++t) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = costmap_ros_->getGlobalFrameID();
      pose.pose.position.x = trajectories.x(i, t);
      pose.pose.position.y = trajectories.y(i, t);
      pose.pose.position.z = 0.0;
      
      // Convert yaw to quaternion
      tf2::Quaternion q;
      q.setRPY(0, 0, trajectories.yaws(i, t));
      pose.pose.orientation = tf2::toMsg(q);
      
      ego_trajectory.push_back(pose);
    }

    // Check collision with all other agents
    double collision_cost = 0.0;
    for (const auto & [agent_name, other_state] : other_agents_) {
      if (check_trajectory_collision(ego_trajectory, other_state, ego_radius)) {
        collision_cost += collision_penalty_;
        RCLCPP_DEBUG(
          logger_,
          "Collision detected with agent '%s' for trajectory sample %zu",
          agent_name.c_str(), i);
      }
    }

    // Add collision cost to total trajectory cost
    data.costs[i] += collision_cost;
  }
}

void MultiAgentInteractionCritic::subscribe_to_agent(const std::string & agent_name)
{
  if (trajectory_subscriptions_.find(agent_name) != trajectory_subscriptions_.end()) {
    return;  // Already subscribed
  }

  try {
    // Build topic name: /<namespace>/mppi_controller/trajectory
    std::string topic_name = "/" + agent_name + "/mppi_controller/trajectory";
    
    // Create subscription with callback that captures agent_name
    auto callback = [this, agent_name](const nav2_msgs::msg::Trajectory::SharedPtr msg) {
      this->trajectory_callback(msg, agent_name);
    };
    
    auto sub = node_->create_subscription<nav2_msgs::msg::Trajectory>(
      topic_name, rclcpp::SensorDataQoS(), callback);
    
    trajectory_subscriptions_[agent_name] = sub;
    
    RCLCPP_INFO(
      logger_,
      "Subscribed to agent '%s' on topic '%s'",
      agent_name.c_str(), topic_name.c_str());
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger_,
      "Failed to subscribe to agent '%s': %s",
      agent_name.c_str(), e.what());
  }
}

void MultiAgentInteractionCritic::trajectory_callback(
  const nav2_msgs::msg::Trajectory::SharedPtr msg,
  const std::string & agent_name)
{
  if (!msg || msg->points.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(agents_mutex_);
  
  OtherAgentState & agent_state = other_agents_[agent_name];
  agent_state.last_update = node_->get_clock()->now();
  agent_state.frame_id = msg->header.frame_id;

  // Extract trajectory endpoint as predicted goal position
  agent_state.pose = extract_trajectory_endpoint(*msg);

  // Estimate velocity from trajectory if enabled
  if (estimate_velocity_) {
    // Use time_from_start from trajectory points or assume 0.05s
    double dt = 0.05;
    if (msg->points.size() >= 2) {
      auto t1 = rclcpp::Duration(msg->points[msg->points.size() - 2].time_from_start).seconds();
      auto t2 = rclcpp::Duration(msg->points.back().time_from_start).seconds();
      dt = t2 - t1;
      if (dt <= 0.0) dt = 0.05;
    }
    agent_state.velocity = estimate_velocity_from_trajectory(*msg, dt);
  }

  // Store full trajectory for collision checking if configured
  if (use_all_trajectory_points_) {
    agent_state.predicted_trajectory.clear();
    agent_state.predicted_trajectory.reserve(msg->points.size());
    
    // Convert trajectory points to PoseStamped
    for (const auto & pt : msg->points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg->header;
      pose.pose = pt.pose;
      agent_state.predicted_trajectory.push_back(pose);
    }
  } else {
    // Use only endpoint
    agent_state.predicted_trajectory.clear();
    agent_state.predicted_trajectory.push_back(agent_state.pose);
  }

  RCLCPP_DEBUG(
    logger_,
    "Updated trajectory for agent '%s': endpoint (%.2f, %.2f), velocity (%.2f, %.2f rad/s)",
    agent_name.c_str(),
    agent_state.pose.pose.position.x,
    agent_state.pose.pose.position.y,
    agent_state.velocity.linear.x,
    agent_state.velocity.angular.z);
}

geometry_msgs::msg::PoseStamped MultiAgentInteractionCritic::extract_trajectory_endpoint(
  const nav2_msgs::msg::Trajectory & trajectory) const
{
  geometry_msgs::msg::PoseStamped endpoint;
  endpoint.header = trajectory.header;
  
  if (trajectory.points.empty()) {
    RCLCPP_WARN(logger_, "Empty trajectory received");
    endpoint.pose.position.x = 0.0;
    endpoint.pose.position.y = 0.0;
    endpoint.pose.position.z = 0.0;
    endpoint.pose.orientation.w = 1.0;
    return endpoint;
  }

  // Extract last point in trajectory
  const auto & last_point = trajectory.points.back();
  endpoint.pose = last_point.pose;
  
  return endpoint;
}

geometry_msgs::msg::Twist MultiAgentInteractionCritic::estimate_velocity_from_trajectory(
  const nav2_msgs::msg::Trajectory & trajectory,
  double dt) const
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 0.0;
  velocity.linear.y = 0.0;
  velocity.linear.z = 0.0;
  velocity.angular.x = 0.0;
  velocity.angular.y = 0.0;
  velocity.angular.z = 0.0;
  
  if (trajectory.points.size() < 2) {
    return velocity;  // Cannot estimate velocity from single point
  }

  // Compute linear velocity from delta between last two points
  const auto & second_last = trajectory.points[trajectory.points.size() - 2];
  const auto & last = trajectory.points.back();
  
  double dx = last.pose.position.x - second_last.pose.position.x;
  double dy = last.pose.position.y - second_last.pose.position.y;
  velocity.linear.x = dx / dt;
  velocity.linear.y = dy / dt;

  // Estimate angular velocity from heading change
  tf2::Quaternion q1, q2;
  tf2::fromMsg(second_last.pose.orientation, q1);
  tf2::fromMsg(last.pose.orientation, q2);
  
  double yaw1 = tf2::getYaw(q1);
  double yaw2 = tf2::getYaw(q2);
  double dtheta = yaw2 - yaw1;
  
  // Normalize angle difference to [-pi, pi]
  while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
  while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
  
  velocity.angular.z = dtheta / dt;
  
  return velocity;
}

bool MultiAgentInteractionCritic::check_trajectory_collision(
  const std::vector<geometry_msgs::msg::PoseStamped> & ego_trajectory,
  const OtherAgentState & other_state,
  double ego_radius) const
{
  // Get combined collision radius
  double collision_radius = ego_radius + other_state.robot_radius + collision_margin_;

  // Check if ego trajectory intersects with any point of other agent's trajectory
  for (const auto & ego_pose : ego_trajectory) {
    for (const auto & other_pose : other_state.predicted_trajectory) {
      double distance = get_distance(ego_pose, other_pose);
      if (distance < collision_radius) {
        RCLCPP_DEBUG(
          logger_,
          "Collision: ego (%.2f, %.2f) vs other (%.2f, %.2f), distance=%.3f < radius=%.3f",
          ego_pose.pose.position.x,
          ego_pose.pose.position.y,
          other_pose.pose.position.x,
          other_pose.pose.position.y,
          distance,
          collision_radius);
        return true;
      }
    }
  }
  return false;
}

double MultiAgentInteractionCritic::get_distance(
  const geometry_msgs::msg::PoseStamped & p1,
  const geometry_msgs::msg::PoseStamped & p2) const
{
  double dx = p1.pose.position.x - p2.pose.position.x;
  double dy = p1.pose.position.y - p2.pose.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

void MultiAgentInteractionCritic::prune_stale_predictions()
{
  std::lock_guard<std::mutex> lock(agents_mutex_);
  auto now = node_->get_clock()->now();
  auto agents_it = other_agents_.begin();
  
  while (agents_it != other_agents_.end()) {
    double time_since_update = (now - agents_it->second.last_update).seconds();
    if (time_since_update > velocity_timeout_) {
      RCLCPP_DEBUG(
        logger_,
        "Pruning stale prediction for agent '%s' (%.1f seconds old)",
        agents_it->first.c_str(), time_since_update);
      agents_it = other_agents_.erase(agents_it);
    } else {
      ++agents_it;
    }
  }
}

double MultiAgentInteractionCritic::get_ego_radius() const
{
  if (costmap_ros_) {
    return costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  }
  return 0.1;  // Default fallback
}

}  // namespace mppi::critics

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mppi::critics::MultiAgentInteractionCritic,
  mppi::critics::CriticFunction)
