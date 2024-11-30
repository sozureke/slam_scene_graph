#include "sg_slam/semantic_graph_node.hpp"
#include "sg_slam/cloud_handler.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace sg_slam {

SemanticGraphNode::SemanticGraphNode()
    : Node("semantic_graph_node"), semantic_graph_() {
    this->declare_parameter<double>("max_radius", 5.0);
    this->declare_parameter<double>("cluster_radius", 1.0);
    this->declare_parameter<int>("delay", 10); 
    this->get_parameter("max_radius", max_radius_);
    this->get_parameter("cluster_radius", cluster_radius_);
    this->get_parameter("delay", delay_);

    slam_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "/slam_data", 10, std::bind(&SemanticGraphNode::slamCallback, this, std::placeholders::_1));

    cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lio_sam/mapping/cloud_registered", 10, std::bind(&SemanticGraphNode::cloudCallback, this, std::placeholders::_1));

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/semantic_graph_markers", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(delay_), std::bind(&SemanticGraphNode::publishGraphMarkers, this));

        RCLCPP_INFO(this->get_logger(), "SemanticGraphNode initialized with max_radius: %f, cluster_radius: %f", max_radius_, cluster_radius_);
}

void SemanticGraphNode::slamCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    robot_position_ = {msg->position.x, msg->position.y};
    RCLCPP_INFO(this->get_logger(), "Received SLAM data: x = %f, y = %f", robot_position_.first, robot_position_.second);
    semantic_graph_.removeOldNodes(robot_position_, max_radius_);
}

void SemanticGraphNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received cloud data.");

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (*iter_z > 0.0 && *iter_z < 5.0) {
            NodeProperties props;
            props.object_type = "PointCloud";
            props.coordinates = {*iter_x, *iter_y};
            semantic_graph_.addNode(props);
        }
    }
    semantic_graph_.clusterNodes(cluster_radius_);
}

void SemanticGraphNode::publishGraphMarkers() {
    visualization_msgs::msg::MarkerArray marker_array;

    // Удаление предыдущих маркеров
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = this->now();
    marker_array.markers.push_back(delete_marker);

    int id = 0;
    for (auto vertex : boost::make_iterator_range(boost::vertices(semantic_graph_.getGraph()))) {
        const auto& properties = semantic_graph_.getGraph()[vertex];

        visualization_msgs::msg::Marker node_marker;
        node_marker.header.frame_id = "map";
        node_marker.header.stamp = this->now();
        node_marker.ns = "nodes";
        node_marker.id = id++;
        node_marker.type = visualization_msgs::msg::Marker::SPHERE;
        node_marker.action = visualization_msgs::msg::Marker::ADD;
        node_marker.pose.position.x = properties.coordinates.first;
        node_marker.pose.position.y = properties.coordinates.second;
        node_marker.pose.position.z = 0.0;
        node_marker.scale.x = 0.15;
        node_marker.scale.y = 0.15;
        node_marker.scale.z = 0.15;
        node_marker.color.a = 1.0;
        node_marker.color.r = 0.0;
        node_marker.color.g = 1.0;
        node_marker.color.b = 0.0;

        visualization_msgs::msg::Marker text_marker = node_marker;
        text_marker.id = id++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.text = properties.object_type;
        text_marker.pose.position.z = 0.3;
        text_marker.scale.z = 0.2;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;

        marker_array.markers.push_back(node_marker);
        marker_array.markers.push_back(text_marker);
    }

    marker_publisher_->publish(marker_array);
    RCLCPP_INFO(this->get_logger(), "Markers published.");
}

}

