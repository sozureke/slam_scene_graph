#include "sg_slam/semantic_graph_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>

namespace sg_slam {

SemanticGraphNode::SemanticGraphNode()
    : Node("semantic_graph_node") {    
    this->declare_parameter<double>("max_radius", 7.0);
    this->declare_parameter<double>("cluster_radius", 0.6);
    this->declare_parameter<int>("min_points_per_cluster", 15);
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

    for (auto vertex : boost::make_iterator_range(boost::vertices(semantic_graph_.getGraph()))) {
        const auto& properties = semantic_graph_.getGraph()[vertex];

        visualization_msgs::msg::Marker object_marker;
        object_marker.header.frame_id = "map";
        object_marker.header.stamp = this->now();
        object_marker.ns = "classified_objects";
        object_marker.id = id++;
        object_marker.type = visualization_msgs::msg::Marker::SPHERE;
        object_marker.pose.position.x = properties.coordinates.x;
        object_marker.pose.position.y = properties.coordinates.y;
        object_marker.pose.position.z = properties.coordinates.z;
        object_marker.scale.x = 0.3;
        object_marker.scale.y = 0.3;
        object_marker.scale.z = 0.3;
        object_marker.color.a = 1.0;

        if (properties.object_type == "Wall") {
            object_marker.color.r = 1.0;
            object_marker.color.g = 0.0;
            object_marker.color.b = 0.0;
        } else if (properties.object_type == "Door") {
            object_marker.color.r = 0.0;
            object_marker.color.g = 1.0;
            object_marker.color.b = 0.0;
        } else if (properties.object_type == "Obstacle") {
            object_marker.color.r = 0.0;
            object_marker.color.g = 0.0;
            object_marker.color.b = 1.0;
        } else {
            object_marker.color.r = 1.0;
            object_marker.color.g = 1.0;
            object_marker.color.b = 1.0;
        }

        marker_array.markers.push_back(object_marker);

        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = this->now();
        text_marker.ns = "classified_objects_text";
        text_marker.id = id + 1000;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.pose.position.x = properties.coordinates.x;
        text_marker.pose.position.y = properties.coordinates.y;
        text_marker.pose.position.z = properties.coordinates.z + 0.5;
        text_marker.scale.z = 0.15;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;

        text_marker.text = "Type: " + properties.object_type +
                           "\nWidth: " + std::to_string(properties.dimensions.width) +
                           "\nHeight: " + std::to_string(properties.dimensions.height) +
                           "\nLength: " + std::to_string(properties.dimensions.length);

        marker_array.markers.push_back(text_marker);
    }

    for (auto edge : boost::make_iterator_range(boost::edges(semantic_graph_.getGraph()))) {
    auto source = boost::source(edge, semantic_graph_.getGraph());
    auto target = boost::target(edge, semantic_graph_.getGraph());

    const auto& source_pos = semantic_graph_.getGraph()[source].coordinates;
    const auto& target_pos = semantic_graph_.getGraph()[target].coordinates;
    const auto& relation = semantic_graph_.getGraph()[edge].relation;

    visualization_msgs::msg::Marker edge_marker;
    edge_marker.header.frame_id = "map";
    edge_marker.header.stamp = this->now();
    edge_marker.ns = "relationships";
    edge_marker.id = id++;
    edge_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    edge_marker.scale.x = 0.05;

    if (relation == "parallel") {
        edge_marker.color.r = 0.0;
        edge_marker.color.g = 1.0;
        edge_marker.color.b = 0.0; 
    } else if (relation == "perpendicular") {
        edge_marker.color.r = 0.0;
        edge_marker.color.g = 0.0;
        edge_marker.color.b = 1.0; 
    } else if (relation == "attached") {
        edge_marker.color.r = 1.0;
        edge_marker.color.g = 0.5;
        edge_marker.color.b = 0.0; 
    }

    edge_marker.color.a = 1.0;

    geometry_msgs::msg::Point p1, p2;
    p1.x = source_pos.x;
    p1.y = source_pos.y;
    p1.z = source_pos.z;
    p2.x = target_pos.x;
    p2.y = target_pos.y;
    p2.z = target_pos.z;

    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);

    RCLCPP_INFO(this->get_logger(), "Publishing edge marker: Source (%f, %f, %f) -> Target (%f, %f, %f)",
                p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);

    marker_array.markers.push_back(edge_marker);
}
    marker_publisher_->publish(marker_array);
}
}