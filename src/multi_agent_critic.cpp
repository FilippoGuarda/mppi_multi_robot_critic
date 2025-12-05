u// Copyright (c) 2024 Your Organization
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <algorithm>

#include "nav2_mppi_controller/critics/multi_agent_interaction_critic.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

void MultiAgentInteractionCritic::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
  const std::string & parent_name,
  const std::string & name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros,
  const std::shared_ptr<nav2_mppi_controller::ParametersHandler> & parameters_handler)
{
  parent_ = parent;
  name_ = name;
  parent_name_ = parent_name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  parameters_handler_ = parameters_handler;
  logger_ = rclcpp::get_logger("MPPIController").get_child(name);

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

  // ========== Load Parameters ==========
  try {
    parameters_handler_->get_param(
      name_ + ".collision_penalty", collision_penalty_, 1000000.0);
    parameters_handler_->get_param(
      name_ + ".collision_margin", collision_margin_, 0.1);
    parameters_handler_->get_param(
      name_ + ".use_all_trajectory_points", use_all_trajectory_points_, false);
    parameters_handler_->get_param(
      name_ + ".estimate_velocity", estimate_velocity_, true);
    parameters_handler_->get_param(
      name_ + ".max_agent_distance", max_agent_distance_, 10.0);
    parameters_handler_->get_param(
      name_ + ".velocity_timeout", velocity_timeout_, 1.0);
    parameters_handler_->get_param(
      name_ + ".enabled", enabled_, true);
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Failed to load parameter: %s", e.what());
  }

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

void MultiAgentInteractionCritic::initialize()
{
  RCLCPP_INFO(logger_, "Initializing MultiAgentInteractionCritic");

  // Note: Agent subscriptions are created dynamically when trajectories are received
  // This allows the critic to adapt to runtime robot discovery
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

  // Iterate over all trajectories in the batch
  for (size_t i = 0; i < data.costs.size(); ++i) {
    // Extract trajectory for this sample
    std::vector<geometry_msgs::msg::PoseStamped> ego_trajectory;
    ego_trajectory.reserve(data.trajectory_points.size());

    // The trajectory data structure in Nav2 MPPI stores trajectories as:
    // data.trajectory contains [batch_size][time_steps][state_dim]
    // We need to extract timesteps for sample i

    for (size_t t = 0; t < data.trajectory_points.size(); ++t) {
      auto pose = data.trajectory_points[t];
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
    // Build topic name: /<agent_namespace>/mppi_controller/trajectory
    std::string topic_name = "/" + agent_name + "/mppi_controller/trajectory";

    // Create subscription with callback that captures agent_name
    auto callback = [this, agent_name](const nav2_msgs::msg::MPPITrajectory::SharedPtr msg) {
      this->trajectory_callback(msg, agent_name);
    };

    auto sub = node_->create_subscription<nav2_msgs::msg::MPPITrajectory>(
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
  const nav2_msgs::msg::MPPITrajectory::SharedPtr msg,
  const std::string & agent_name)
{
  if (!msg || msg->trajectory.empty()) {
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
    // Use model_dt from trajectory message (if available) or assume 0.05s
    double dt = 0.05;
    if (msg->model_dt > 0.0) {
      dt = msg->model_dt;
    }
    agent_state.velocity = estimate_velocity_from_trajectory(*msg, dt);
  }

  // Store full trajectory for collision checking if configured
  if (use_all_trajectory_points_) {
    agent_state.predicted_trajectory.clear();
    agent_state.predicted_trajectory.reserve(msg->trajectory.size());

    // Convert trajectory points to PoseStamped
    for (const auto & pt : msg->trajectory) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg->header;
      pose.pose.position.x = pt.x;
      pose.pose.position.y = pt.y;
      pose.pose.position.z = pt.z;

      // Heading stored in pt.theta (for 2D navigation)
      tf2::Quaternion q;
      q.setRPY(0, 0, pt.theta);
      pose.pose.orientation = tf2::toMsg(q);

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
  const nav2_msgs::msg::MPPITrajectory & trajectory) const
{
  geometry_msgs::msg::PoseStamped endpoint;
  endpoint.header = trajectory.header;

  if (trajectory.trajectory.empty()) {
    RCLCPP_WARN(logger_, "Empty trajectory received");
    endpoint.pose.position.x = 0.0;
    endpoint.pose.position.y = 0.0;
    endpoint.pose.position.z = 0.0;
    endpoint.pose.orientation.w = 1.0;
    return endpoint;
  }

  // Extract last point in trajectory
  const auto & last_point = trajectory.trajectory.back();

  endpoint.pose.position.x = last_point.x;
  endpoint.pose.position.y = last_point.y;
  endpoint.pose.position.z = last_point.z;

  // Convert heading (theta) to quaternion
  // Assumes 2D navigation: roll=0, pitch=0, yaw=theta
  tf2::Quaternion q;
  q.setRPY(0, 0, last_point.theta);
  endpoint.pose.orientation = tf2::toMsg(q);

  return endpoint;
}

geometry_msgs::msg::Twist MultiAgentInteractionCritic::estimate_velocity_from_trajectory(
  const nav2_msgs::msg::MPPITrajectory & trajectory,
  double dt) const
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 0.0;
  velocity.linear.y = 0.0;
  velocity.linear.z = 0.0;
  velocity.angular.x = 0.0;
  velocity.angular.y = 0.0;
  velocity.angular.z = 0.0;

  if (trajectory.trajectory.size() < 2) {
    return velocity;  // Cannot estimate velocity from single point
  }

  // Compute linear velocity from delta between last two points
  const auto & second_last = trajectory.trajectory[trajectory.trajectory.size() - 2];
  const auto & last = trajectory.trajectory.back();

  double dx = last.x - second_last.x;
  double dy = last.y - second_last.y;

  velocity.linear.x = dx / dt;
  velocity.linear.y = dy / dt;

  // Estimate angular velocity from heading change
  double dtheta = last.theta - second_last.theta;
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
  // Extract robot footprint radius from costmap
  // The costmap inflation radius approximates the robot radius
  if (costmap_ros_) {
    return costmap_ros_->getLayeredCostmap()->getInscribedRadius();
  }
  return 0.1;  // Default fallback
}

}  // namespace mppi::critics

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mppi::critics::MultiAgentInteractionCritic,
  nav2_mppi_controller::CriticFunction)