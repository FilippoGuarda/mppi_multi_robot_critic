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
  
  std::vector<geometry_msg::msg::Point> predictTrajectoryFromPath();
  double computeCollisionCosts();
  bool checkCollisions();

  
}


} //namespace mppi::critics

#endif // NAV2_MPPI_MULTI_ROBOT_CRITIC