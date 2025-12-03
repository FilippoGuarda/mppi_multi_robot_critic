/**
 * @file trajectory_aware_critic.cpp
 * @brief Implementation of TrajectoryAwareCritic for multi-robot MPPI coordination
 *
 * Copyright (c) 2025 - University of Bologna
 * Licensed under the Apache License, Version 2.0
 */


#include "mppi_multi_robot_critic/trajectory_aware_critic.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
#include <algorithm>

namespace mppi::critics
{

void TrajectoryAwareCritic::initialize(
  rclcpp::Node::WeakPtr parent,
  const std::string & critic_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
  ParametersHandler * param_handler)
{
  parent_node_ = parent;
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf_buffer;
  logger_ = rclcpp::get_logger(critic_name);

  auto node = parent_node_.lock();
  if (!node) {
    throw std::runtime_error("Parent node expired");
  }

  auto getParam = param_handler->getParamGetter(critic_name);

  // Core collision parameters
  getParam(collision_cost_, "collision_cost", 1000000.0f);
  getParam(collision_margin_, "collision_margin", 0.15f);
  getParam(cost_weight_, "cost_weight", 1.5f);

  // Trajectory handling parameters
  getParam(max_trajectory_age_, "max_trajectory_age", 2.0f);
  getParam(use_full_trajectory_, "use_full_trajectory", true);
  getParam(max_distance_to_consider_, "max_distance_to_consider", 5.0f);
  getParam(lookahead_distance_, "lookahead_distance", 2.0f);
  getParam(trajectory_interpolation_step_, "trajectory_interpolation_step", 0.1f);
  getParam(default_robot_radius_, "default_robot_radius", 0.25f);
  getParam(consider_footprint_, "consider_footprint", true);
  getParam(local_robot_id_, "local_robot_id", "ego");

  // Topic patterns
  getParam(trajectory_topic_patterns_, "trajectory_topic_patterns",
    std::vector<std::string>{"/robot_*/controller_server/optimal_trajectory"});
  getParam(goal_topic_patterns_, "goal_topic_patterns",
    std::vector<std::string>{"/robot_*/controller_server/goal"});

  // COLREGs parameters
  getParam(regulation_violation_cost_, "regulation_violation_cost", 500.0f);
  getParam(starboard_detection_radius_, "starboard_detection_radius", 1.0f);
  getParam(starboard_detection_angle_, "starboard_detection_angle", 0.349f);
  getParam(head_on_angle_margin_, "head_on_angle_margin", 0.349f);
  getParam(relative_velocity_tolerance_, "relative_velocity_tolerance", 0.1f);
  getParam(min_velocity_threshold_, "min_velocity_threshold", 0.1f);

  // Debug parameters
  getParam(publish_debug_markers_, "publish_debug_markers", false);

  RCLCPP_INFO(logger_, "Initialized TrajectoryAwareCritic: %s", critic_name.c_str());
  RCLCPP_INFO(logger_, "Listening for trajectory patterns: %zu patterns", 
    trajectory_topic_patterns_.size());

  // Start discovering trajectories
  discoverAndSubscribeToTrajectories();
}

void TrajectoryAwareCritic::score(CriticData & data)
{
  // Clean up stale trajectories periodically
  cleanupStaleTrajectories();

  // Get current robot pose and velocity
  auto robot_pose = data.state.pose;
  auto robot_velocity = data.state.velocity;

  // Lock trajectory access
  std::lock_guard<std::mutex> lock(trajectory_mutex_);

  // Score each MPPI sample
  for (size_t sample = 0; sample < data.costs.rows(); ++sample) {
    float sample_cost = 0.0f;

    // Check collision against each remote trajectory
    for (const auto & [robot_id, other_trajectory] : robot_trajectories_) {
      // Skip self-checking
      if (robot_id == local_robot_id_) {
        continue;
      }

      // Check distance - only consider nearby robots
      auto [min_dist, _] = getClosestPointOnTrajectory(
        other_trajectory, robot_pose.position);
      if (min_dist > max_distance_to_consider_) {
        continue;
      }

      // Check collision at each timestep
      for (size_t time_step = 0; time_step < data.costs.cols(); ++time_step) {
        // Trajectory collision cost
        float traj_cost = computeTrajectoryCollisionCost(
          data.trajectories[sample],
          time_step,
          robot_pose,
          other_trajectory);
        sample_cost += traj_cost;

        // COLREGs-like regulation cost
        float reg_cost = computeRegulationCost(
          data.trajectories[sample],
          time_step,
          robot_pose,
          other_trajectory);
        sample_cost += reg_cost;
      }
    }

    // Apply cost weight
    data.costs(sample, 0) += cost_weight_ * sample_cost;
  }

  // Optional debug visualization
  if (publish_debug_markers_) {
    publishDebugMarkers(data);
  }
}

void TrajectoryAwareCritic::trajectoryCallback(
  const std::string & robot_id,
  const nav2_msgs::msg::Trajectory::SharedPtr msg)
{
  if (!msg || msg->trajectory.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(trajectory_mutex_);

  // Create or update trajectory entry
  RobotTrajectory robot_traj;
  robot_traj.robot_id = robot_id;
  robot_traj.last_update_time = rclcpp::Clock().now();
  robot_traj.robot_radius = default_robot_radius_;
  robot_traj.has_full_trajectory = true;

  // Extract trajectory points from nav2_msgs::msg::Trajectory
  // NOTE: Changed from nav_msgs::msg::Path to nav2_msgs::msg::Trajectory
  robot_traj.trajectory_points.clear();

  for (const auto & pose : msg->trajectory) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = msg->header;
    pose_stamped.pose = pose;
    robot_traj.trajectory_points.push_back(pose_stamped);
  }

  // Store goal as last point
  if (!robot_traj.trajectory_points.empty()) {
    robot_traj.goal_pose = robot_traj.trajectory_points.back().pose;
  }

  robot_trajectories_[robot_id] = robot_traj;

  RCLCPP_DEBUG(logger_, "Updated trajectory for robot %s with %zu points",
    robot_id.c_str(), robot_traj.trajectory_points.size());
}

void TrajectoryAwareCritic::discoverAndSubscribeToTrajectories()
{
  auto node = parent_node_.lock();
  if (!node) {
    return;
  }

  // This would typically discover topics dynamically
  // For now, we'll use explicit topic patterns
  // A full implementation would use ros2 topic discovery APIs

  RCLCPP_DEBUG(logger_, "Trajectory topic discovery initialized");
}

void TrajectoryAwareCritic::subscribeToTrajectoryTopic(
  const std::string & topic_name,
  const std::string & robot_id)
{
  auto node = parent_node_.lock();
  if (!node) {
    return;
  }

  auto callback = [this, robot_id](const nav2_msgs::msg::Trajectory::SharedPtr msg) {
    this->trajectoryCallback(robot_id, msg);
  };

  auto subscription = node->create_subscription<nav2_msgs::msg::Trajectory>(
    topic_name, rclcpp::QoS(10), callback);

  trajectory_subscriptions_[robot_id] = subscription;

  RCLCPP_INFO(logger_, "Subscribed to trajectory topic: %s for robot: %s",
    topic_name.c_str(), robot_id.c_str());
}

std::string TrajectoryAwareCritic::extractRobotIdFromTopic(
  const std::string & topic)
{
  // Extract from format: /robot_0/controller_server/optimal_trajectory
  size_t first_slash = topic.find('/');
  if (first_slash == std::string::npos || first_slash + 1 >= topic.length()) {
    return "unknown";
  }

  size_t second_slash = topic.find('/', first_slash + 1);
  if (second_slash == std::string::npos) {
    return "unknown";
  }

  return topic.substr(first_slash + 1, second_slash - first_slash - 1);
}

float TrajectoryAwareCritic::computeTrajectoryCollisionCost(
  const Eigen::ArrayXXf & trajectory,
  size_t time_step,
  const geometry_msgs::msg::Pose & pose,
  const RobotTrajectory & other_trajectory)
{
  if (time_step >= trajectory.cols() || other_trajectory.trajectory_points.empty()) {
    return 0.0f;
  }

  // Get ego position at this timestep
  float ego_x = trajectory(0, time_step, 0);
  float ego_y = trajectory(0, time_step, 1);

  float min_distance = std::numeric_limits<float>::max();

  // Check distance to each point in other's trajectory
  for (const auto & other_pose : other_trajectory.trajectory_points) {
    float dx = ego_x - other_pose.pose.position.x;
    float dy = ego_y - other_pose.pose.position.y;
    float distance = std::sqrt(dx * dx + dy * dy);

    min_distance = std::min(min_distance, distance);
  }

  // Calculate collision cost based on distance
  float safety_distance = collision_margin_ + other_trajectory.robot_radius;

  if (min_distance < safety_distance) {
    // Hard collision penalty
    return collision_cost_;
  } else if (min_distance < safety_distance + lookahead_distance_) {
    // Gradient penalty as we approach
    float gradient = (min_distance - safety_distance) / lookahead_distance_;
    return collision_cost_ * (1.0f - gradient);
  }

  return 0.0f;
}

float TrajectoryAwareCritic::computeRegulationCost(
  const Eigen::ArrayXXf & trajectory,
  size_t time_step,
  const geometry_msgs::msg::Pose & ego_pose,
  const RobotTrajectory & other_trajectory)
{
  // Check if COLREGs rules apply
  bool starboard = detectStarboardApproach(ego_pose, {}, other_trajectory);
  bool head_on = detectHeadOnApproach(ego_pose, {}, other_trajectory);

  float cost = 0.0f;

  // If other robot has starboard right-of-way and we're not yielding
  if (starboard) {
    // Check if our trajectory is moving into their space
    if (time_step < trajectory.cols()) {
      float ego_x = trajectory(0, time_step, 0);
      float ego_y = trajectory(0, time_step, 1);

      // If we're getting closer to other's trajectory
      auto [dist, _] = getClosestPointOnTrajectory(
        other_trajectory, {ego_x, ego_y, 0.0f});

      if (dist < max_distance_to_consider_) {
        cost += regulation_violation_cost_;
      }
    }
  }

  // Head-on scenarios - both should yield equally
  if (head_on) {
    cost += regulation_violation_cost_ * 0.5f;
  }

  return cost;
}

std::pair<float, float> TrajectoryAwareCritic::getClosestPointOnTrajectory(
  const RobotTrajectory & trajectory,
  const geometry_msgs::msg::Point & point)
{
  float min_distance = std::numeric_limits<float>::max();
  float closest_parameter = 0.0f;

  for (size_t i = 0; i < trajectory.trajectory_points.size(); ++i) {
    const auto & pose = trajectory.trajectory_points[i];
    float dx = point.x - pose.pose.position.x;
    float dy = point.y - pose.pose.position.y;
    float distance = std::sqrt(dx * dx + dy * dy);

    if (distance < min_distance) {
      min_distance = distance;
      closest_parameter = static_cast<float>(i);
    }
  }

  return {min_distance, closest_parameter};
}

float TrajectoryAwareCritic::getDistanceToCollision(
  const geometry_msgs::msg::Pose & ego_pose,
  const Eigen::ArrayXXf & ego_sample,
  size_t time_step,
  const RobotTrajectory & other_trajectory)
{
  if (time_step >= ego_sample.cols()) {
    return std::numeric_limits<float>::max();
  }

  float ego_x = ego_sample(0, time_step, 0);
  float ego_y = ego_sample(0, time_step, 1);

  auto [min_dist, _] = getClosestPointOnTrajectory(
    other_trajectory, {ego_x, ego_y, 0.0f});

  return min_dist - collision_margin_ - other_trajectory.robot_radius;
}

bool TrajectoryAwareCritic::detectStarboardApproach(
  const geometry_msgs::msg::Pose & ego_pose,
  const geometry_msgs::msg::Twist & ego_velocity,
  const RobotTrajectory & other_trajectory)
{
  if (other_trajectory.trajectory_points.empty()) {
    return false;
  }

  // Get other robot's position and heading
  const auto & other_pose = other_trajectory.trajectory_points[0].pose;

  float dx = other_pose.position.x - ego_pose.position.x;
  float dy = other_pose.position.y - ego_pose.position.y;
  float distance = std::sqrt(dx * dx + dy * dy);

  if (distance > starboard_detection_radius_) {
    return false;
  }

  // Get ego heading
  auto ego_quat = tf2::Quaternion(
    ego_pose.orientation.x,
    ego_pose.orientation.y,
    ego_pose.orientation.z,
    ego_pose.orientation.w);
  float ego_yaw = tf2::getYaw(ego_quat);

  // Calculate bearing to other robot
  float bearing = std::atan2(dy, dx);

  // Check if other is on starboard (right) side
  // Starboard is typically 0-90 degrees to the right
  float angle_diff = bearing - ego_yaw;

  // Normalize to [-pi, pi]
  while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
  while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

  // Starboard is -90 to 0 degrees (-pi/2 to 0)
  return angle_diff >= -M_PI_2 - starboard_detection_angle_ &&
         angle_diff <= -M_PI_2 + starboard_detection_angle_;
}

bool TrajectoryAwareCritic::detectHeadOnApproach(
  const geometry_msgs::msg::Pose & ego_pose,
  const geometry_msgs::msg::Twist & ego_velocity,
  const RobotTrajectory & other_trajectory)
{
  if (other_trajectory.trajectory_points.empty()) {
    return false;
  }

  const auto & other_pose = other_trajectory.trajectory_points[0].pose;

  float dx = other_pose.position.x - ego_pose.position.x;
  float dy = other_pose.position.y - ego_pose.position.y;
  float distance = std::sqrt(dx * dx + dy * dy);

  if (distance > starboard_detection_radius_) {
    return false;
  }

  // Get both headings
  auto ego_quat = tf2::Quaternion(
    ego_pose.orientation.x,
    ego_pose.orientation.y,
    ego_pose.orientation.z,
    ego_pose.orientation.w);
  float ego_yaw = tf2::getYaw(ego_quat);

  auto other_quat = tf2::Quaternion(
    other_pose.orientation.x,
    other_pose.orientation.y,
    other_pose.orientation.z,
    other_pose.orientation.w);
  float other_yaw = tf2::getYaw(other_quat);

  // Calculate relative heading
  float heading_diff = other_yaw - ego_yaw;

  // Normalize to [-pi, pi]
  while (heading_diff > M_PI) heading_diff -= 2 * M_PI;
  while (heading_diff < -M_PI) heading_diff += 2 * M_PI;

  // Head-on is approaching from opposite direction
  return std::abs(heading_diff) > (M_PI - head_on_angle_margin_);
}

void TrajectoryAwareCritic::cleanupStaleTrajectories()
{
  std::lock_guard<std::mutex> lock(trajectory_mutex_);

  auto now = rclcpp::Clock().now();
  std::vector<std::string> to_remove;

  for (const auto & [robot_id, trajectory] : robot_trajectories_) {
    double age_seconds = (now - trajectory.last_update_time).seconds();
    if (age_seconds > max_trajectory_age_) {
      to_remove.push_back(robot_id);
    }
  }

  for (const auto & robot_id : to_remove) {
    robot_trajectories_.erase(robot_id);
    RCLCPP_DEBUG(logger_, "Removed stale trajectory for robot %s", robot_id.c_str());
  }
}

void TrajectoryAwareCritic::publishDebugMarkers(const CriticData & data)
{
  // Optional: Implement RViz marker publishing for visualization
  // This would show trajectories, collision zones, etc.
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::TrajectoryAwareCritic,
  mppi::critics::CriticFunction)