// Copyright (c) 2024 Your Organization
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

#ifndef MPPI_CRITICS__MULTI_AGENT_INTERACTION_CRITIC_HPP_
#define MPPI_CRITICS__MULTI_AGENT_INTERACTION_CRITIC_CRITIC_HPP_

#include <string>
#include <memory>
#include <vector>
#include <map>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/msg/mppi_trajectory.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "nav2_mppi_controller/critics/critic_function.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi::critics
{

/**
 * @struct OtherAgentState
 * @brief Stores predicted state of another robot agent
 */
struct OtherAgentState
{
  std::string frame_id;                  // Robot's frame ID
  geometry_msgs::msg::PoseStamped pose;  // Last known pose
  geometry_msgs::msg::Twist velocity;    // Estimated velocity (from trajectory delta)
  std::vector<geometry_msgs::msg::PoseStamped> predicted_trajectory;  // Predicted future positions
  rclcpp::Time last_update;              // Timestamp of last update
  double robot_radius = 0.1;             // Footprint radius for collision checking
};

/**
 * @class MultiAgentInteractionCritic
 * @brief Critic that penalizes collisions with other agents using trajectory predictions
 *
 * Implements dynamic collision detection as described in Trevisan et al. ICRA 2023:
 * "Multi-Agent Path Integral Control for Interaction-Aware Motion Planning"
 *
 * This critic:
 * 1. Subscribes to other robots' planned trajectories
 * 2. Extracts final trajectory point as predicted goal position
 * 3. Estimates velocity from trajectory deltas
 * 4. Applies discontinuous collision penalty when ego trajectory overlaps with other agents
 */
class MultiAgentInteractionCritic : public CriticFunction
{
public:
  /**
   * @brief Configure critic parameters
   */
  void configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    const std::string & parent_name,
    const std::string & name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros,
    const std::shared_ptr<nav2_mppi_controller::ParametersHandler> & parameters_handler) override;

  /**
   * @brief Initialize critic (called after configure)
   */
  void initialize() override;

  /**
   * @brief Score trajectories based on collision with other agents
   * @param data Critic data containing trajectories to score
   */
  void score(CriticData & data) override;

private:
  // ========== Configuration Parameters ==========
  double collision_penalty_ = 1000000.0;  // High penalty for collisions (from paper: C_collision)
  double collision_margin_ = 0.1;         // Safety margin around robot radius
  bool use_all_trajectory_points_ = false; // Use full trajectory or just endpoint
  bool estimate_velocity_ = true;          // Compute velocity from trajectory delta
  double max_agent_distance_ = 10.0;      // Ignore agents further than this
  double velocity_timeout_ = 1.0;         // Seconds before velocity prediction expires

  // ========== ROS Subscriptions & State ==========
  std::map<std::string, OtherAgentState> other_agents_;  // Map of agent_name -> state
  std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> trajectory_subscriptions_;
  std::mutex agents_mutex_;  // Protects other_agents_ map

  rclcpp::Node::SharedPtr node_;
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
    const nav2_msgs::msg::MPPITrajectory::SharedPtr msg,
    const std::string & agent_name);

  /**
   * @brief Extract final trajectory point from MPPI trajectory message
   * @param trajectory The trajectory message
   * @return Last pose in trajectory (or current pose if trajectory empty)
   */
  geometry_msgs::msg::PoseStamped extract_trajectory_endpoint(
    const nav2_msgs::msg::MPPITrajectory & trajectory) const;

  /**
   * @brief Estimate velocity from consecutive trajectory points
   * @param trajectory MPPI trajectory message
   * @param dt Time step between trajectory points
   * @return Estimated velocity (linear and angular)
   */
  geometry_msgs::msg::Twist estimate_velocity_from_trajectory(
    const nav2_msgs::msg::MPPITrajectory & trajectory,
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