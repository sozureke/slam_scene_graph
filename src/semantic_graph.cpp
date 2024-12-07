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

    std::vector<Vertex> vertices_to_remove;
    for (auto vp = boost::vertices(graph_); vp.first != vp.second; ++vp.first) {
        vertices_to_remove.push_back(*vp.first);
    }
    for (auto it = vertices_to_remove.rbegin(); it != vertices_to_remove.rend(); ++it) {
        boost::clear_vertex(*it, graph_);
        boost::remove_vertex(*it, graph_);
    }
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

        addNode(cluster_properties);
    }

    RCLCPP_INFO(rclcpp::get_logger("sg_slam"), "Clusters added to the graph successfully.");
}



sg_slam::ObjectDimensions::ObjectDimensions(double w, double h, double l)
    : width(w), height(h), length(l) {}

}  
