#include "sg_slam/semantic_graph.hpp"
#include "sg_slam/slam_handler.hpp"
#include "sg_slam/cloud_handler.hpp"
#include "sg_slam/marker_publisher.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("semantic_graph_node");

    double max_radius = node->declare_parameter<double>("max_radius", 5.0);
    double cluster_radius = node->declare_parameter<double>("cluster_radius", 1.0);
    double z_min = node->declare_parameter<double>("z_min", 0.0);
    double z_max = node->declare_parameter<double>("z_max", 5.0);
    double voxel_leaf_size = node->declare_parameter<double>("voxel_leaf_size", 0.1);
    int publish_rate = node->declare_parameter<int>("publish_rate", 10);

    // Semantic Graph
    sg_slam::SemanticGraph graph;

    sg_slam::SlamHandler slam_handler(graph, node, max_radius);
    sg_slam::CloudHandler cloud_handler(graph, node, z_min, z_max, voxel_leaf_size, cluster_radius);
    sg_slam::MarkerPublisher marker_publisher(graph, node, publish_rate);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
