#include "sg_slam/semantic_graph.hpp"
#include <cmath>
#include <unordered_set>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

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

void SemanticGraph::updateNodePosition(Vertex node, const Position& new_coordinates) {
    graph_[node].coordinates = new_coordinates;
}

const Graph& SemanticGraph::getGraph() const {
    return graph_;
}

void SemanticGraph::removeOldNodes(const Position& robot_position, double max_radius) {
    double max_radius_squared = max_radius * max_radius;
    std::vector<Vertex> to_remove;
    for (auto vp = boost::vertices(graph_); vp.first != vp.second; ++vp.first) {
        Vertex vertex = *vp.first;
        const Position& node_pos = graph_[vertex].coordinates;
        double dx = node_pos.x - robot_position.x;
        double dy = node_pos.y - robot_position.y;
        double dz = node_pos.z - robot_position.z;
        double distance_squared = dx * dx + dy * dy + dz * dz;
        if (distance_squared > max_radius_squared) {
            to_remove.push_back(vertex);
        }
    }
    for (auto it = to_remove.rbegin(); it != to_remove.rend(); ++it) {
        Vertex vertex = *it;
        boost::clear_vertex(vertex, graph_);
        boost::remove_vertex(vertex, graph_);
    }
}

void SemanticGraph::clearGraph() {
    graph_.clear();
}

double SemanticGraph::calculatePointDensity(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<int>& indices) {
    if (indices.empty()) return 0.0;

    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloud, indices, min_pt, max_pt);

    double volume = (max_pt[0] - min_pt[0]) * (max_pt[1] - min_pt[1]) * (max_pt[2] - min_pt[2]);
    if (volume <= 0.0) return 0.0;

    return static_cast<double>(indices.size()) / volume;
}

void SemanticGraph::filterStableClusters(const std::vector<NodeProperties>& current_clusters) {
    std::unordered_map<int, NodeProperties> updated_stable_clusters;

    for (const auto& cluster : current_clusters) {
        bool merged = false;
        for (auto& [id, stable_cluster] : stable_clusters) {
            double distance = std::sqrt(
                std::pow(cluster.coordinates.x - stable_cluster.coordinates.x, 2) +
                std::pow(cluster.coordinates.y - stable_cluster.coordinates.y, 2) +
                std::pow(cluster.coordinates.z - stable_cluster.coordinates.z, 2));

            if (distance < 0.5) {
                stable_cluster.coordinates.x = (stable_cluster.coordinates.x + cluster.coordinates.x) / 2;
                stable_cluster.coordinates.y = (stable_cluster.coordinates.y + cluster.coordinates.y) / 2;
                stable_cluster.coordinates.z = (stable_cluster.coordinates.z + cluster.coordinates.z) / 2;
                merged = true;
                break;
            }
        }

        if (!merged) {
            updated_stable_clusters[updated_stable_clusters.size()] = cluster;
        }
    }

    stable_clusters = std::move(updated_stable_clusters);
}

void SemanticGraph::publishStableClusters() {
    for (const auto& [id, cluster] : stable_clusters) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "classified_objects";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.pose.position.x = cluster.coordinates.x;
        marker.pose.position.y = cluster.coordinates.y;
        marker.pose.position.z = cluster.coordinates.z;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;

        if (cluster.object_type == "Wall") {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else if (cluster.object_type == "Door") {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        } else if (cluster.object_type == "Obstacle") {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        } else {
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
        }

        marker.color.a = 1.0;
        addNode(cluster);
    }
}

void SemanticGraph::adaptiveClusterNodes(double base_cluster_radius, int min_points_per_cluster) {
    if (boost::num_vertices(graph_) < 2) {
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::unordered_map<int, Vertex> vertex_map;
    int index = 0;

    for (auto vertex : boost::make_iterator_range(boost::vertices(graph_))) {
        cloud->push_back(pcl::PointXYZ(graph_[vertex].coordinates.x, graph_[vertex].coordinates.y, graph_[vertex].coordinates.z));
        vertex_map[index++] = vertex;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(base_cluster_radius);
    ec.setMinClusterSize(min_points_per_cluster);
    ec.setMaxClusterSize(cloud->size());
    ec.setSearchMethod(tree);

    std::vector<pcl::PointIndices> cluster_indices;
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    RCLCPP_INFO(rclcpp::get_logger("sg_slam"), "Number of clusters found: %zu", cluster_indices.size());

    std::vector<NodeProperties> new_clusters;

    for (const auto& cluster : cluster_indices) {
        if (cluster.indices.size() < static_cast<size_t>(min_points_per_cluster)) {
            continue;
        }

        double density = calculatePointDensity(cloud, cluster.indices);
        if (density < 0.1) {
            continue;
        }

        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud, cluster.indices, min_pt, max_pt);

        NodeProperties cluster_properties;
        cluster_properties.object_type = "Cluster";
        cluster_properties.coordinates = Position{(min_pt[0] + max_pt[0]) / 2, (min_pt[1] + max_pt[1]) / 2, (min_pt[2] + max_pt[2]) / 2};
        cluster_properties.dimensions.width = max_pt[0] - min_pt[0];
        cluster_properties.dimensions.height = max_pt[1] - min_pt[1];
        cluster_properties.dimensions.length = max_pt[2] - min_pt[2];

        new_clusters.push_back(cluster_properties);
    }

    filterStableClusters(new_clusters);
    publishStableClusters();
}

void SemanticGraph::clusterNodes(double cluster_radius, int min_points_per_cluster) {
    if (boost::num_vertices(graph_) < 2) {
        RCLCPP_WARN(rclcpp::get_logger("sg_slam"), "Not enough points to form clusters.");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::unordered_map<int, Vertex> vertex_map;
    int index = 0;

    for (auto vertex : boost::make_iterator_range(boost::vertices(graph_))) {
        cloud->push_back(pcl::PointXYZ(graph_[vertex].coordinates.x, graph_[vertex].coordinates.y, graph_[vertex].coordinates.z));
        vertex_map[index++] = vertex;
    }

    if (cloud->empty()) {
        RCLCPP_WARN(rclcpp::get_logger("sg_slam"), "Point cloud is empty, cannot perform clustering.");
        return;
    }

    graph_.clear();
    RCLCPP_INFO(rclcpp::get_logger("sg_slam"), "Cleared old graph.");

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_radius);
    ec.setMinClusterSize(min_points_per_cluster);
    ec.setMaxClusterSize(cloud->size());
    ec.setSearchMethod(tree);

    std::vector<pcl::PointIndices> cluster_indices;
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    RCLCPP_INFO(rclcpp::get_logger("sg_slam"), "Found %zu clusters.", cluster_indices.size());

    for (const auto& cluster : cluster_indices) {
        if (cluster.indices.size() < static_cast<size_t>(min_points_per_cluster)) {
            continue;
        }

        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud, cluster.indices, min_pt, max_pt);

        NodeProperties cluster_properties;
        cluster_properties.object_type = "Cluster";
        cluster_properties.coordinates = Position{
            (min_pt[0] + max_pt[0]) / 2,
            (min_pt[1] + max_pt[1]) / 2,
            (min_pt[2] + max_pt[2]) / 2
        };
        cluster_properties.dimensions.width = max_pt[0] - min_pt[0];
        cluster_properties.dimensions.height = max_pt[1] - min_pt[1];
        cluster_properties.dimensions.length = max_pt[2] - min_pt[2];

        cluster_properties.object_type = classifyObject(cluster_properties);
        addNode(cluster_properties);
    }
}

std::string SemanticGraph::classifyObject(const NodeProperties& cluster) {
    const auto& dims = cluster.dimensions;

    if (dims.height > 2.5 && dims.width < 0.5) {
        return "Wall";
    } else if (dims.height < 1.0 && dims.width < 1.0 && dims.length < 1.0) {
        return "Obstacle";
    } else if (dims.height > 2.0 && dims.width > 0.8 && dims.width < 1.2) {
        return "Door";
    }

    return "Unknown";
}

} 

sg_slam::ObjectDimensions::ObjectDimensions(double w, double h, double l)
    : width(w), height(h), length(l) {}
