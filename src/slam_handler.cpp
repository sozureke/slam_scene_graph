#include "sg_slam/slam_handler.hpp"

namespace sg_slam {

SlamHandler::SlamHandler(SemanticGraph& graph, rclcpp::Node::SharedPtr node, double max_radius)
    : graph_(graph), max_radius_(max_radius) {
    subscription_ = node->create_subscription<geometry_msgs::msg::Pose>(
        "/slam_data", 10, std::bind(&SlamHandler::slamCallback, this, std::placeholders::_1));
}

void SlamHandler::slamCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    robot_position_ = {msg->position.x, msg->position.y};
    graph_.removeOldNodes(robot_position_, max_radius_);
}

} 
