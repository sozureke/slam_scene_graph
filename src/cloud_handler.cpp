#include "sg_slam/cloud_handler.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace sg_slam {

CloudHandler::CloudHandler(SemanticGraph& graph, double& cluster_radius, int& min_points_per_cluster)
    : graph_(graph), cluster_radius_(cluster_radius), min_points_per_cluster_(min_points_per_cluster) {}

void CloudHandler::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received cloud data.");

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (*iter_z > 0.0 && *iter_z < 5.0) {
            NodeProperties props;
            props.object_type = "PointCloud";
            props.coordinates = {*iter_x, *iter_y};
            graph_.addNode(props);
        }
    }

    graph_.clusterNodes(cluster_radius_, min_points_per_cluster_);
}

void CloudHandler::setClusterRadius(double radius) {
    cluster_radius_ = radius;
}

void CloudHandler::setMinPointsPerCluster(int points) {
    min_points_per_cluster_ = points;
}

}
