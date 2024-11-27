#ifndef SG_SLAM_SLAM_HANDLER_HPP_
#define SG_SLAM_SLAM_HANDLER_HPP_

#include "sg_slam/semantic_graph.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace sg_slam {

class SlamHandler {
public:
    SlamHandler(SemanticGraph& graph, rclcpp::Node::SharedPtr node, double max_radius);
    void slamCallback(const geometry_msgs::msg::Pose::SharedPtr msg);

private:
    SemanticGraph& graph_;
    double max_radius_;
    std::pair<double, double> robot_position_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
};

}

#endif
