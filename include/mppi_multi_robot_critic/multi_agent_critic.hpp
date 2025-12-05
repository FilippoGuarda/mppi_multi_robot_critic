#ifndef MPPI_CRITICS__MULTI_AGENT_INTERACTION_CRITIC_HPP_
#define MPPI_CRITICS__MULTI_AGENT_INTERACTION_CRITIC_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/msg/trajectory.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi::critics
{

/**
 * @struct OtherAgentState
 * @brief Stores predicted state of another robot agent
 */
struct OtherAgentState
{
  std::string frame_id;
  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::Twist velocity;
  std::vector<geometry_msgs::msg::PoseStamped> predicted_trajectory;
  rclcpp::Time last_update;
  double robot_radius = 0.1;
};

/**
 * @class MultiAgentInteractionCritic
 * @brief Critic that penalizes collisions with other agents using trajectory predictions
 */
class MultiAgentInteractionCritic : public CriticFunction
{
public:
  /**
   * @brief Initialize critic (called after on_configure)
   */
  void initialize() override;

  /**
   * @brief Score trajectories based on collision with other agents
   * @param data Critic data containing trajectories to score
   */
  void score(CriticData & data) override;

private:
  // Configuration Parameters
  double collision_penalty_ = 1000000.0;
  double collision_margin_ = 0.1;
  bool use_all_trajectory_points_ = false;
  bool estimate_velocity_ = true;
  double max_agent_distance_ = 10.0;
  double velocity_timeout_ = 1.0;
  std::vector<std::string> other_robot_namespaces_;

  // ROS Subscriptions & State
  std::map<std::string, OtherAgentState> other_agents_;
  std::map<std::string, rclcpp::Subscription<nav2_msgs::msg::Trajectory>::SharedPtr> trajectory_subscriptions_;
  std::mutex agents_mutex_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string robot_namespace_;

  // ========== Implementation Methods ==========

  /**
   * @brief Create trajectory subscription for another agent
   * @param agent_name Robot namespace (e.g., "robot_1")
   */
  void subscribe_to_agent(const std::string & agent_name);

  /**
   * @brief Trajectory message callback for subscribed agents
   * @param msg MPPI trajectory from another robot
   * @param agent_name Name of robot that published the trajectory
   */
  void trajectory_callback(
    const nav2_msgs::msg::Trajectory::SharedPtr msg,
    const std::string & agent_name);

  /**
   * @brief Extract final trajectory point from MPPI trajectory message
   * @param trajectory The trajectory message
   * @return Last pose in trajectory (or current pose if trajectory empty)
   */
  geometry_msgs::msg::PoseStamped extract_trajectory_endpoint(
    const nav2_msgs::msg::Trajectory & trajectory) const;

  /**
   * @brief Estimate velocity from consecutive trajectory points
   * @param trajectory MPPI trajectory message
   * @param dt Time step between trajectory points
   * @return Estimated velocity (linear and angular)
   */
  geometry_msgs::msg::Twist estimate_velocity_from_trajectory(
    const nav2_msgs::msg::Trajectory & trajectory,
    double dt) const;

  /**
   * @brief Check for collision between ego trajectory and other agent trajectory
   * @param ego_trajectory Current robot's trajectory
   * @param other_state Predicted state of other robot
   * @param ego_radius Radius of ego robot
   * @return true if collision detected, false otherwise
   */
  bool check_trajectory_collision(
    const std::vector<geometry_msgs::msg::PoseStamped> & ego_trajectory,
    const OtherAgentState & other_state,
    double ego_radius) const;

  /**
   * @brief Compute 2D distance between two poses
   * @param p1 First pose
   * @param p2 Second pose
   * @return Euclidean distance
   */
  double get_distance(
    const geometry_msgs::msg::PoseStamped & p1,
    const geometry_msgs::msg::PoseStamped & p2) const;

  /**
   * @brief Prune stale predictions (older than velocity_timeout_)
   */
  void prune_stale_predictions();

  /**
   * @brief Get radius of ego robot from costmap
   * @return Robot radius
   */
  double get_ego_radius() const;
};

}  // namespace mppi::critics

#endif  // MPPI_CRITICS__MULTI_AGENT_INTERACTION_CRITIC_HPP_
