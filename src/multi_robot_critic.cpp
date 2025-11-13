#include "multi_robot_critic.hpp"
#include <cmath>
#include <algorithm>

namespace mppi::critics {

void FleetCollisionCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  
  getParam(collision_cost_, "collision_cost", 100000.0);
  getParam(repulsion_cost_weight_, "repulsion_weight", 10.0);
  getParam(safety_radius_, "safety_radius", 0.6);
  getParam(collision_margin_, "collision_margin", 0.1);
  getParam(min_distance_threshold_, "min_distance_threshold", 2.0);
  getParam(near_goal_distance_, "near_goal_distance", 0.5);
  getParam(exclude_collision_trajectories_, "exclude_collision_trajectories", true);
  getParam(enabled_, "enabled", true);
  getParam(trajectory_point_step_, "trajectory_point_step", 3);
  getParam(fleet_paths_topic_, "fleet_paths_topic", std::string("/fleet/paths"));
  
  auto node = parent_.lock();
  if (node) {
    fleet_paths_sub_ = node->create_subscription<nav_msgs::msg::Path>(
      fleet_paths_topic_,
      rclcpp::QoS(10),
      std::bind(&FleetCollisionCritic::fleetPathsCallback, this, std::placeholders::_1));
      
    RCLCPP_INFO(logger_, 
      "FleetCollisionCritic initialized. Subscribing to %s", 
      fleet_paths_topic_.c_str());
  }
}

void MultiRobotCritic::multiPathsCallback(const nav_msg::Path::SharedPtr msg){

}

void FleetCollisionCritic::fleetPathsCallback(const nav_msgs::msg::Path::ShardPtr msg){

}

std::vector<geometry_msgs::msg::Point>
FleetCollisionCritic::predictTrajectoryFromPath(
  const nav_msgs::msg::Path & path,
  const rclcpp::Time & current_time,
  double prediction_horizon,
  double dt)
{

}

bool FleetCollisionCritic::checkCollision(
  const geometry_msgs::msg::Pose & ego_pose,
  const geometry_msgs::msg::Point & neighbor_pos,
  double combined_radius)
{
  
}

void MultiRobotCritic::computeCollisionCosts(
  const xt::xtensor<float, 2> & ego_poses,
  const std::vector<geometry_msgs::msg::Point> & neighbor_positions,
  double neighbor_radius,
  size_t time_step_idx)
{
  
}

void MultiRobotCritic::score(CriticData & data){

}


} //namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    mppi::critics::MultiRobotCritic,
    mppi::critics::CriticFunction
)