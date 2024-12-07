#include "sg_slam/semantic_graph_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>

namespace sg_slam {

SemanticGraphNode::SemanticGraphNode()
    : Node("semantic_graph_node") {
    this->declare_parameter<double>("max_radius", 5.0);
    this->declare_parameter<double>("cluster_radius", 0.5);
    this->declare_parameter<int>("min_points_per_cluster", 20);
    this->declare_parameter<int>("delay", 10);

    this->get_parameter("max_radius", max_radius_);
    this->get_parameter("cluster_radius", cluster_radius_);
    this->get_parameter("min_points_per_cluster", min_points_per_cluster_);
    this->get_parameter("delay", delay_);

    cloud_handler_ = std::make_shared<CloudHandler>(semantic_graph_, cluster_radius_, min_points_per_cluster_, graph_mutex_);

    slam_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/slam_data", 10, std::bind(&SemanticGraphNode::slamCallback, this, std::placeholders::_1));

    cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lio_sam/mapping/cloud_registered", 10, std::bind(&SemanticGraphNode::cloudCallback, this, std::placeholders::_1));

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/semantic_graph_markers", 10);

    timer_ = this->create_wall_timer(
        std::chrono::seconds(delay_),
        std::bind(&SemanticGraphNode::publishGraphMarkers, this));

    RCLCPP_INFO(this->get_logger(), "SemanticGraphNode initialized with max_radius: %f, cluster_radius: %f, min_points_per_cluster: %d",
                max_radius_, cluster_radius_, min_points_per_cluster_);
}

void SemanticGraphNode::slamCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    robot_position_.x = msg->position.x;
    robot_position_.y = msg->position.y;
    robot_position_.z = msg->position.z;

    RCLCPP_INFO(this->get_logger(), "Received SLAM data: x = %f, y = %f, z = %f", robot_position_.x, robot_position_.y, robot_position_.z);

    semantic_graph_.removeOldNodes(robot_position_, max_radius_);
}

void SemanticGraphNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    cloud_handler_->cloudCallback(msg);
}

void SemanticGraphNode::publishGraphMarkers() {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = this->now();
    marker_array.markers.push_back(delete_marker);

    int id = 0;
    size_t vertex_count = boost::num_vertices(semantic_graph_.getGraph());
    RCLCPP_INFO(this->get_logger(), "Publishing markers for %zu vertices.", vertex_count);

    for (auto vertex : boost::make_iterator_range(boost::vertices(semantic_graph_.getGraph()))) {
        const auto& properties = semantic_graph_.getGraph()[vertex];

        visualization_msgs::msg::Marker cluster_marker;
        cluster_marker.header.frame_id = "map";
        cluster_marker.header.stamp = this->now();
        cluster_marker.ns = "clusters";
        cluster_marker.id = id++;
        cluster_marker.type = visualization_msgs::msg::Marker::SPHERE;
        cluster_marker.action = visualization_msgs::msg::Marker::ADD;

        cluster_marker.pose.position.x = properties.coordinates.x;
        cluster_marker.pose.position.y = properties.coordinates.y;
        cluster_marker.pose.position.z = properties.coordinates.z;

        cluster_marker.scale.x = 0.3; 
        cluster_marker.scale.y = 0.3;
        cluster_marker.scale.z = 0.3;
        cluster_marker.color.a = 1.0;
        cluster_marker.color.r = 1.0;
        cluster_marker.color.g = 0.0;
        cluster_marker.color.b = 0.0;

        marker_array.markers.push_back(cluster_marker);
    }

    marker_publisher_->publish(marker_array);
    RCLCPP_INFO(this->get_logger(), "Cluster markers published.");
}
} 
