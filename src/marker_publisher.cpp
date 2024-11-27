#include "sg_slam/marker_publisher.hpp"

namespace sg_slam {

MarkerPublisher::MarkerPublisher(SemanticGraph& graph, rclcpp::Node::SharedPtr node, int publish_rate)
    : graph_(graph), node_(node) {
    publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/semantic_graph_markers", 10);
    timer_ = node_->create_wall_timer(
        std::chrono::seconds(publish_rate),
        std::bind(&MarkerPublisher::publishMarkers, this));
}

void MarkerPublisher::publishMarkers() {
    visualization_msgs::msg::MarkerArray marker_array;

    // Clear old markers
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = node_->now();
    marker_array.markers.push_back(delete_marker);

    int id = 0;
    for (auto vertex : boost::make_iterator_range(boost::vertices(graph_.getGraph()))) {
        const auto& properties = graph_.getGraph()[vertex];

        // Node marker
        visualization_msgs::msg::Marker node_marker;
        node_marker.header.frame_id = "map";
        node_marker.header.stamp = node_->now();
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

        // Text marker
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

    publisher_->publish(marker_array);
    RCLCPP_INFO(node_->get_logger(), "Markers published.");
}

}
