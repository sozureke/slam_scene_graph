#ifndef SG_SLAM_MARKER_PUBLISHER_HPP_
#define SG_SLAM_MARKER_PUBLISHER_HPP_

#include "sg_slam/semantic_graph.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace sg_slam {

class MarkerPublisher {
public:
    MarkerPublisher(SemanticGraph& graph, rclcpp::Node::SharedPtr node, int publish_rate = 10);
    void publishMarkers();

private:
    SemanticGraph& graph_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  

#endif 
