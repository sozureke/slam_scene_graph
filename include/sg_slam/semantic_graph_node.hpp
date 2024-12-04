#ifndef SG_SLAM_SEMANTIC_GRAPH_NODE_HPP_
#define SG_SLAM_SEMANTIC_GRAPH_NODE_HPP_

#include "sg_slam/semantic_graph.hpp"
#include "sg_slam/cloud_handler.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <memory>
#include <mutex>

namespace sg_slam {

class SemanticGraphNode : public rclcpp::Node {
public:
    SemanticGraphNode();

private:
    void slamCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publishGraphMarkers();

    double max_radius_;
    double cluster_radius_;
    int delay_;
    int min_points_per_cluster_;

    std::shared_ptr<CloudHandler> cloud_handler_;
    SemanticGraph semantic_graph_;
    Position robot_position_;

    std::mutex graph_mutex_;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr slam_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace sg_slam

#endif
