#include "sg_slam/cloud_handler.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sg_slam {

CloudHandler::CloudHandler(SemanticGraph& graph, double cluster_radius, int min_points_per_cluster, std::mutex& graph_mutex)
    : graph_(graph), cluster_radius_(cluster_radius), min_points_per_cluster_(min_points_per_cluster), graph_mutex_(graph_mutex) {}

void CloudHandler::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("sg_slam"), "Received cloud data.");

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    size_t points_added = 0;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (*iter_z > 0.0 && *iter_z < 5.0) {
            NodeProperties props;
            props.object_type = "PointCloud";
            props.coordinates = Position{*iter_x, *iter_y, *iter_z};
            graph_.addNode(props);
            points_added++;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("sg_slam"), "Added %zu points to the graph.", points_added);
    graph_.clusterNodes(cluster_radius_, min_points_per_cluster_);
}

void CloudHandler::setClusterRadius(double radius) {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    cluster_radius_ = radius;
}

void CloudHandler::setMinPointsPerCluster(int points) {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    min_points_per_cluster_ = points;
}

}