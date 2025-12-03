// nav2_interaction_critic/include/nav2_interaction_critic/agent_state_monitor.hpp
#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <Eigen/Core>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/string.hpp"

namespace nav2_interaction_critic
{

struct VelocityEstimate
{
  Eigen::Vector2f linear;
  float angular;
  rclcpp::Time timestamp;
};

class AgentStateMonitor
{
public:
  /**
   * @brief Initialize the monitor
   */
  void initialize(
    rclcpp::Node::SharedPtr node,
    const std::vector<std::string> & agent_frame_ids,
    const std::vector<std::string> & agent_twist_topics);

  /**
   * @brief Get agent position from TF
   */
  bool getAgentPose(
    const std::string & frame_id,
    Eigen::Vector3f & pose);  // x, y, heading

  /**
   * @brief Get agent velocity estimate (uses velocity history)
   */
  bool getAgentVelocity(
    const std::string & frame_id,
    Eigen::Vector2f & linear_vel,
    float & angular_vel);

  /**
   * @brief Get agent velocity from direct subscription
   */
  bool getAgentVelocityDirect(
    const std::string & frame_id,
    Eigen::Vector2f & linear_vel,
    float & angular_vel);

private:
  /**
   * @brief Estimate velocity from pose history using constant velocity model
   */
  void estimateVelocityFromPoseHistory(
    const std::string & frame_id,
    Eigen::Vector2f & linear_vel);

  /**
   * @brief Twist message callback
   */
  void twistCallback(
    const std::string & frame_id,
    const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Velocity estimates from direct subscription
  std::map<std::string, VelocityEstimate> velocity_estimates_;
  std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr> 
    twist_subscriptions_;
  
  // Pose history for velocity estimation (sliding window)
  static constexpr size_t POSE_HISTORY_SIZE = 10;
  struct PoseHistory
  {
    std::vector<Eigen::Vector3f> poses;
    std::vector<rclcpp::Time> timestamps;
  };
  std::map<std::string, PoseHistory> pose_history_;
};

}  // namespace nav2_interaction_critic
