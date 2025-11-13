#include "multi_robot_critic.hpp"
#include <cmath>
#include <algorithm>

namespace mppi::critics {

void MultiRobotCritic::initialize(){

}

void MultiRobotCritic::multiPathsCallback(const nav_msg::Path::SharedPtr msg){

}

void MultiRobotCritic::computeCollisionCosts(){

}

void MultiRobotCritic::checkCollisions(){

}

void MultiRobotCritic::score(CriticData & data){

}


} //namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    mppi::critics::MultiRobotCritic,
    mppi::critics::CriticFunction
)