#ifndef NAV2_MPPI_MULTI_ROBOT_CRITIC_HPP_
#define NAV2_MPPI_MULTI_ROBOT_CRITIC_HPP_

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <mutex>

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mppi::critics {

class MultiRobotCritic : public CriticFunction
{
public:
  MultiRobotCritic() = default;

  void initialize() ovveride;
  void score(CriticData & data) ovverride;

protected:
  
  void multiPathsCallback(const nav_msg::Path::SharedPtr msg);
  
  std::vector<geometry_msgs::msg::Point> predictTrajectoryFromPath(
    const nav_msg::msg::Path & path,
    const rclcpp::Time & current_time,
    double prediction_horizon,
    double dt
  );
    
  double computeCollisionCosts(
    const xt::xtensor<float, 2> & self_pose,
    const std::vector<geometry_msg::msg::Point> & neighbor_positions,
    double neighbor_radius,
    size_t time_step_idx
  );

  bool checkCollision(
    const geometry_msg::msg::Pose & self_pose,
    const geometry_msg::msg::Point & neighbor_pos,
    double combined_radius
  );

  // Params

  double collistion_cost_{1000000.0};
  double repulsion_cost_weight_{10.0};
  double safety_radius_{0.6};
  double collision_margin_{0.1};
  double near_goal_distance_{0.5};
  bool exclude_collision_trajectories_{true};
  bool enabled_{true};
  int trajectory_point_step_{3};

  std::unordered_map<std::string, nav_msgs::msg::Path> robot_paths_;
  std::string fleet_paths_topic_;

  std::unordered_map<std::string, std::vector<geometry_msg::msg::Point>> predicted_trajectories_;
}


} //namespace mppi::critics

#endif // NAV2_MPPI_MULTI_ROBOT_CRITIC