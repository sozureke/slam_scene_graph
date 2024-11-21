#include "sg_slam/semantic_graph.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <unordered_set>
#include <vector>

namespace sg_slam {

SemanticGraph::SemanticGraph() {}

Vertex SemanticGraph::addNode(const NodeProperties& properties) {
    Vertex v = boost::add_vertex(graph_);
    graph_[v] = properties;
    return v;
}

void SemanticGraph::addEdge(Vertex node1, Vertex node2) {
    boost::add_edge(node1, node2, graph_);
}

void SemanticGraph::updateNodePosition(Vertex node, const std::pair<double, double>& new_coordinates) {
    graph_[node].coordinates = new_coordinates;
}

const Graph& SemanticGraph::getGraph() const {
    return graph_;
}

void SemanticGraph::removeOldNodes(const std::pair<double, double>& robot_position, double max_radius) {
    std::vector<Vertex> to_remove;
    for (auto vertex : boost::make_iterator_range(boost::vertices(graph_))) {
        double distance = std::sqrt(std::pow(graph_[vertex].coordinates.first - robot_position.first, 2) +
                                    std::pow(graph_[vertex].coordinates.second - robot_position.second, 2));
        if (distance > max_radius) {
            to_remove.push_back(vertex);
        }
    }
    for (auto vertex : to_remove) {
        boost::clear_vertex(vertex, graph_);
        boost::remove_vertex(vertex, graph_);
    }
}

void SemanticGraph::clusterNodes(double cluster_radius) {
    if (boost::num_vertices(graph_) < 2) {
        return; // Нет необходимости кластеризовать, если меньше двух узлов
    }

    // Создание K-d дерева из координат вершин графа
    pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::unordered_map<int, Vertex> vertex_map;
    int index = 0;
    for (auto vertex : boost::make_iterator_range(boost::vertices(graph_))) {
        pcl::PointXYZ point(graph_[vertex].coordinates.first, graph_[vertex].coordinates.second, 0.0);
        cloud->push_back(point);
        vertex_map[index++] = vertex;
    }

    kd_tree.setInputCloud(cloud);

    std::unordered_set<Vertex> to_remove;
    std::vector<int> nearest_indices;
    std::vector<float> nearest_distances;

    for (size_t i = 0; i < cloud->size(); ++i) {
        if (to_remove.find(vertex_map[i]) != to_remove.end()) {
            continue; // Если уже удалено, пропускаем
        }

        pcl::PointXYZ search_point = cloud->points[i];
        kd_tree.radiusSearch(search_point, cluster_radius, nearest_indices, nearest_distances);

        for (size_t j = 1; j < nearest_indices.size(); ++j) { // Пропустить самую первую точку (это текущая точка)
            Vertex neighbor_vertex = vertex_map[nearest_indices[j]];
            if (to_remove.find(neighbor_vertex) == to_remove.end()) {
                to_remove.insert(neighbor_vertex);
            }
        }
    }

    // Удаление из графа
    for (auto vertex : to_remove) {
        boost::clear_vertex(vertex, graph_);
        boost::remove_vertex(vertex, graph_);
    }
}

class SemanticGraphNode : public rclcpp::Node {
public:
    SemanticGraphNode() : Node("semantic_graph_node"), semantic_graph_() {
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

private:
    void slamCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        robot_position_ = {msg->position.x, msg->position.y};
        RCLCPP_INFO(this->get_logger(), "Received SLAM data: x = %f, y = %f", robot_position_.first, robot_position_.second);
        semantic_graph_.removeOldNodes(robot_position_, max_radius_);
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
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

    void publishGraphMarkers() {
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

    SemanticGraph semantic_graph_;
    std::pair<double, double> robot_position_;
    double max_radius_;
    double cluster_radius_;
    int delay_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr slam_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
};

}  // namespace sg_slam

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sg_slam::SemanticGraphNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
