// nav2_interaction_critic/include/nav2_interaction_critic/interaction_aware_critic.hpp
#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <Eigen/Core>

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/critic_data.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_interaction_critic
{

struct AgentState
{
  Eigen::Vector2f position;
  float heading;
  Eigen::Vector2f velocity;
  float radius = 0.25f;
  float max_speed = 1.7f;
};

struct CostWeights
{
  float static_obstacle_weight = 100.0f;
  float rotation_weight = 5.0f;
  float tracking_weight = 10.0f;
  float speed_weight = 50.0f;
  float dynamic_collision_weight = 100.0f;
  float regulation_weight = 50.0f;
};

/**
 * @class InteractionAwareCritic
 * @brief MPPI critic for multi-agent interaction-aware motion planning
 * 
 * Implements the cost functions from:
 * "Multi-Agent Path Integral Control for Interaction-Aware Motion Planning 
 *  in Urban Canals" - Trevisan et al., ICRA 2023
 */
class InteractionAwareCritic : public mppi::critics::CriticFunction
{
public:
  virtual ~InteractionAwareCritic() = default;

  void initialize() override;
  void score(mppi::CriticData & data) override;

protected:
  // Update tracked agents' states from TF
  void updateOtherAgentsState(const std::string & frame_id);

  // Predict agent's local goal using constant velocity model
  Eigen::Vector2f predictLocalGoal(
    const AgentState & agent,
    float prediction_horizon);

  // Cost computation functions
  float computeTrackingCost(
    const xt::xtensor<float, 1> & x_traj,
    const xt::xtensor<float, 1> & y_traj,
    const Eigen::Vector2f & goal);

  float computeRotationCost(
    const xt::xtensor<float, 1> & x_traj,
    const xt::xtensor<float, 1> & y_traj);

  float computeStaticCollisionCost(
    const xt::xtensor<float, 1> & x_traj,
    const xt::xtensor<float, 1> & y_traj);

  float computeRegulationCost(
    const xt::xtensor<float, 1> & x_traj,
    const xt::xtensor<float, 1> & y_traj,
    const AgentState & other_agent);

  // Regulation violation checks (COLREGs)
  bool isRightOfWayViolation(
    const Eigen::Vector2f & robot_pos,
    const Eigen::Vector2f & robot_vel,
    const AgentState & other_agent);

  bool isHeadOnEncounter(
    const Eigen::Vector2f & robot_vel,
    const Eigen::Vector2f & other_vel);

  bool detectDynamicCollision(
    const Eigen::Vector2f & robot_pos,
    const AgentState & other_agent);

  Eigen::Vector2f extractLocalGoal(const mppi::models::Path & path);

  // TF utilities
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // Cost weights
  CostWeights weights_;

  // Configuration parameters
  float collision_penalty_ = 1000.0f;
  float rotation_penalty_slow_ = 1.0f;
  float rotation_penalty_fast_ = 2.0f;
  float speed_threshold_ = 0.5f;
  float max_speed_ = 1.7f;
  float angular_margin_ = 0.2f;
  float right_of_way_radius_ = 2.0f;
  float min_agent_velocity_ = 0.5f;

  // Tracked agents
  std::map<std::string, AgentState> other_agents_;
  std::vector<std::string> tracked_agent_names_;

  unsigned char lethal_cost_ = 253;
};

}  // namespace nav2_interaction_critic