/**
 * @file trajectory_aware_critic.hpp
 * @brief MPPI critic for multi-robot coordination using published optimal trajectories
 *
 * This critic uses trajectories published by other robots (via nav2_msgs::msg::Trajectory)
 * to perform collision-aware planning. It subscribes to all robot trajectories and checks
 * collisions against them during MPPI trajectory optimization.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_mppi_controller/critic_function.hpp"


namespace mppi::critics
{

/**
 * @struct RobotTrajectory
 * @brief Stores trajectory data for a remote robot
 */
struct RobotTrajectory
{
  std::string robot_id;
  std::vector<geometry_msgs::msg::PoseStamped> trajectory_points;
  geometry_msgs::msg::Pose goal_pose;
  float robot_radius;
  rclcpp::Time last_update_time;
  bool has_full_trajectory;
};

/**
 * @class TrajectoryAwareCritic
 * @brief MPPI critic for multi-robot collision avoidance using shared trajectories
 *
 * This critic enables decentralized multi-robot coordination with communication by:
 * 1. Subscribing to optimal trajectories published by other robots
 * 2. Checking MPPI candidate trajectories for collisions
 * 3. Returning costs to penalize collision-prone paths
 *
 * Implementation details:
 * - Subscribes to nav2_msgs::msg::Trajectory on configurable topic patterns
 * - Uses frame-by-frame collision checking
 * - Implements COLREGs-like navigation rules (right-of-way detection)
 * - Supports dynamic obstacle discovery via wildcard subscriptions
 */
class TrajectoryAwareCritic : public CriticFunction
{
public:
  /**
   * @brief Initialize the critic
   */
  void initialize(
    rclcpp::Node::WeakPtr parent,
    const std::string & critic_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    ParametersHandler * param_handler) override;

  /**
   * @brief Score trajectories based on collision risks
   * @param data CriticData containing trajectory samples and metadata
   */
  void score(CriticData & data) override;

private:
  // ======================================================================
  // Subscribers and Data Storage
  // ======================================================================

  /// Subscriptions to remote robot trajectories
  std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> trajectory_subscriptions_;

  /// Cached trajectories from other robots
  std::map<std::string, RobotTrajectory> robot_trajectories_;

  /// Mutex for thread-safe trajectory access
  std::mutex trajectory_mutex_;

  // ======================================================================
  // Configuration Parameters
  // ======================================================================

  /// Collision cost penalty
  float collision_cost_;

  /// Minimum safety distance around obstacles (meters)
  float collision_margin_;

  /// Cost weight relative to other critics
  float cost_weight_;

  /// Maximum age of trajectory before discarding (seconds)
  float max_trajectory_age_;

  /// Whether to use full trajectory or just endpoint
  bool use_full_trajectory_;

  /// Maximum distance to consider robots for collision (meters)
  float max_distance_to_consider_;

  /// How far ahead in trajectory to check (meters)
  float lookahead_distance_;

  /// Interpolation step size for trajectory checking (meters)
  float trajectory_interpolation_step_;

  /// Default radius for remote robots (meters)
  float default_robot_radius_;

  /// Whether to use full robot footprint vs point
  bool consider_footprint_;

  /// Robot ID to avoid self-checking
  std::string local_robot_id_;

  /// Topic patterns to search for trajectories
  std::vector<std::string> trajectory_topic_patterns_;

  /// Goal topic patterns to search for
  std::vector<std::string> goal_topic_patterns_;

  /// COLREGs regulation violation cost
  float regulation_violation_cost_;

  /// Detection radius for starboard right-of-way (meters)
  float starboard_detection_radius_;

  /// Angular margin for starboard detection (radians)
  float starboard_detection_angle_;

  /// Angular margin for head-on detection (radians)
  float head_on_angle_margin_;

  /// Velocity threshold to consider robot dynamic (m/s)
  float relative_velocity_tolerance_;

  /// Minimum velocity to consider robot moving (m/s)
  float min_velocity_threshold_;

  /// Enable RViz debug markers
  bool publish_debug_markers_;

  // ======================================================================
  // ROS Interfaces
  // ======================================================================

  rclcpp::Node::WeakPtr parent_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_;

  // ======================================================================
  // Internal Methods
  // ======================================================================

  /**
   * @brief Callback for trajectory messages from other robots
   * @param robot_id ID of the robot publishing trajectory
   * @param msg The published trajectory message
   */
  void trajectoryCallback(
    const std::string & robot_id,
    const nav2_msgs::msg::Trajectory::SharedPtr msg);

  /**
   * @brief Discover and subscribe to trajectory topics
   */
  void discoverAndSubscribeToTrajectories();

  /**
   * @brief Subscribe to a specific trajectory topic
   * @param topic_name Full topic name to subscribe to
   * @param robot_id Robot ID extracted from topic
   */
  void subscribeToTrajectoryTopic(
    const std::string & topic_name,
    const std::string & robot_id);

  /**
   * @brief Extract robot ID from topic name
   * @param topic Topic name (e.g., "/robot_0/controller_server/optimal_trajectory")
   * @return Robot ID (e.g., "robot_0")
   */
  std::string extractRobotIdFromTopic(const std::string & topic);

  /**
   * @brief Compute collision cost against a single trajectory
   * @param sample MPPI trajectory sample
   * @param time_step Current time step
   * @param pose Robot current pose
   * @param trajectory Remote robot trajectory
   * @return Collision cost
   */
  float computeTrajectoryCollisionCost(
    const Eigen::ArrayXXf & trajectory,
    size_t time_step,
    const geometry_msgs::msg::Pose & pose,
    const RobotTrajectory & other_trajectory);

  /**
   * @brief Compute COLREGs regulation violation cost
   * @param sample MPPI trajectory sample
   * @param time_step Current time step
   * @param ego_pose Robot pose
   * @param other_trajectory Remote robot trajectory
   * @return Regulation violation cost
   */
  float computeRegulationCost(
    const Eigen::ArrayXXf & trajectory,
    size_t time_step,
    const geometry_msgs::msg::Pose & ego_pose,
    const RobotTrajectory & other_trajectory);

  /**
   * @brief Get the closest point on a trajectory to a given position
   * @param trajectory The trajectory to search
   * @param point The point to find closest approach to
   * @return Pair of (closest_distance, closest_parameter)
   */
  std::pair<float, float> getClosestPointOnTrajectory(
    const RobotTrajectory & trajectory,
    const geometry_msgs::msg::Point & point);

  /**
   * @brief Check if trajectories will collide
   * @param ego_pose Robot pose
   * @param ego_sample MPPI trajectory sample
   * @param time_step Current time step
   * @param other_trajectory Remote trajectory
   * @return Distance to collision (negative if colliding, positive if safe)
   */
  float getDistanceToCollision(
    const geometry_msgs::msg::Pose & ego_pose,
    const Eigen::ArrayXXf & ego_sample,
    size_t time_step,
    const RobotTrajectory & other_trajectory);

  /**
   * @brief Detect if another robot has right-of-way (COLREGs starboard rule)
   * @param ego_pose Robot pose
   * @param ego_velocity Robot velocity
   * @param other_trajectory Remote trajectory
   * @return True if other robot has right-of-way
   */
  bool detectStarboardApproach(
    const geometry_msgs::msg::Pose & ego_pose,
    const geometry_msgs::msg::Twist & ego_velocity,
    const RobotTrajectory & other_trajectory);

  /**
   * @brief Detect head-on approach scenario
   * @param ego_pose Robot pose
   * @param ego_velocity Robot velocity
   * @param other_trajectory Remote trajectory
   * @return True if head-on approach detected
   */
  bool detectHeadOnApproach(
    const geometry_msgs::msg::Pose & ego_pose,
    const geometry_msgs::msg::Twist & ego_velocity,
    const RobotTrajectory & other_trajectory);

  /**
   * @brief Clean up stale trajectories (older than max_trajectory_age_)
   */
  void cleanupStaleTrajectories();

  /**
   * @brief Publish debug markers for visualization
   * @param data CriticData for context
   */
  void publishDebugMarkers(const CriticData & data);
};

}  // namespace mppi::critics