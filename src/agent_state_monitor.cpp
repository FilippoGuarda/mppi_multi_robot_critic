// nav2_interaction_critic/src/agent_state_monitor.cpp
#include "mppi_multi_robot_critic/agent_state_monitor.hpp"

#include <cmath>

namespace nav2_interaction_critic
{

void AgentStateMonitor::initialize(
  rclcpp::Node::SharedPtr node,
  const std::vector<std::string> & agent_frame_ids,
  const std::vector<std::string> & agent_twist_topics)
{
  node_ = node;
  
  // Initialize TF buffer
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Initialize velocity estimates
  for (size_t i = 0; i < agent_frame_ids.size(); ++i)
  {
    const auto & frame_id = agent_frame_ids[i];
    velocity_estimates_[frame_id] = {
      {0.0f, 0.0f},
      0.0f,
      node_->get_clock()->now()
    };
    
    // Initialize pose history
    pose_history_[frame_id] = {
      std::vector<Eigen::Vector3f>(),
      std::vector<rclcpp::Time>()
    };
  }
  
  // Subscribe to twist topics if provided
  if (!agent_twist_topics.empty())
  {
    for (size_t i = 0; i < std::min(agent_frame_ids.size(), agent_twist_topics.size()); ++i)
    {
      const auto & frame_id = agent_frame_ids[i];
      const auto & topic = agent_twist_topics[i];
      
      auto callback = [this, frame_id](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
      {
        twistCallback(frame_id, msg);
      };
      
      twist_subscriptions_[frame_id] = 
        node_->create_subscription<geometry_msgs::msg::TwistStamped>(
          topic, rclcpp::SensorDataQoS(), callback);
    }
  }
}

bool AgentStateMonitor::getAgentPose(
  const std::string & frame_id,
  Eigen::Vector3f & pose)  // x, y, heading
{
  try
  {
    geometry_msgs::msg::TransformStamped transform =
      tf_buffer_->lookupTransform("map", frame_id, tf2::TimePointZero);
    
    pose.x() = transform.transform.translation.x;
    pose.y() = transform.transform.translation.y;
    
    // Extract heading from quaternion
    tf2::Quaternion q(
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z,
      transform.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    pose.z() = static_cast<float>(yaw);
    
    // Update pose history
    auto & history = pose_history_[frame_id];
    history.poses.push_back(pose);
    history.timestamps.push_back(node_->get_clock()->now());
    
    if (history.poses.size() > POSE_HISTORY_SIZE)
    {
      history.poses.erase(history.poses.begin());
      history.timestamps.erase(history.timestamps.begin());
    }
    
    return true;
  }
  catch (const tf2::TransformException & ex)
  {
    return false;
  }
}

bool AgentStateMonitor::getAgentVelocity(
  const std::string & frame_id,
  Eigen::Vector2f & linear_vel,
  float & angular_vel)
{
  // Try direct subscription first
  if (getAgentVelocityDirect(frame_id, linear_vel, angular_vel))
  {
    return true;
  }
  
  // Fall back to pose history estimation
  estimateVelocityFromPoseHistory(frame_id, linear_vel);
  angular_vel = 0.0f;
  return linear_vel.norm() > 0.01f;
}

bool AgentStateMonitor::getAgentVelocityDirect(
  const std::string & frame_id,
  Eigen::Vector2f & linear_vel,
  float & angular_vel)
{
  auto it = velocity_estimates_.find(frame_id);
  if (it == velocity_estimates_.end())
  {
    return false;
  }
  
  const auto & estimate = it->second;
  rclcpp::Duration time_since_update = node_->get_clock()->now() - estimate.timestamp;
  
  // If estimate is fresh (< 1 second old)
  if (time_since_update.seconds() < 1.0)
  {
    linear_vel = estimate.linear;
    angular_vel = estimate.angular;
    return true;
  }
  
  return false;
}

void AgentStateMonitor::estimateVelocityFromPoseHistory(
  const std::string & frame_id,
  Eigen::Vector2f & linear_vel)
{
  linear_vel = {0.0f, 0.0f};
  
  auto it = pose_history_.find(frame_id);
  if (it == pose_history_.end() || it->second.poses.size() < 2)
  {
    return;
  }
  
  const auto & history = it->second;
  
  // Use linear regression on recent pose history
  size_t n = history.poses.size();
  
  // Simple: use last two poses
  const auto & prev_pose = history.poses[n - 2];
  const auto & curr_pose = history.poses[n - 1];
  
  double dt = (history.timestamps[n - 1] - history.timestamps[n - 2]).seconds();
  
  if (dt > 0.01)
  {
    linear_vel.x() = static_cast<float>((curr_pose.x() - prev_pose.x()) / dt);
    linear_vel.y() = static_cast<float>((curr_pose.y() - prev_pose.y()) / dt);
  }
}

void AgentStateMonitor::twistCallback(
  const std::string & frame_id,
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  VelocityEstimate estimate;
  estimate.linear.x() = static_cast<float>(msg->twist.linear.x);
  estimate.linear.y() = static_cast<float>(msg->twist.linear.y);
  estimate.angular = static_cast<float>(msg->twist.angular.z);
  estimate.timestamp = node_->get_clock()->now();
  
  velocity_estimates_[frame_id] = estimate;
}

}  // namespace nav2_interaction_critic
