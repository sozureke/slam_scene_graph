#include "sg_slam/semantic_graph.hpp"
#include <cmath>
#include <unordered_set>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

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
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::unordered_map<int, Vertex> vertex_map;
    int index = 0;
    for (auto vertex : boost::make_iterator_range(boost::vertices(graph_))) {
        cloud->emplace_back(pcl::PointXYZ(graph_[vertex].coordinates.x, graph_[vertex].coordinates.y, graph_[vertex].coordinates.z));
        vertex_map[index++] = vertex;
    }

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

    std::unordered_set<Vertex> to_remove;
    for (const auto& cluster : cluster_indices) {
        if (cluster.indices.size() < min_points_per_cluster) {
            continue;
        }
        for (const auto& idx : cluster.indices) {
            to_remove.insert(vertex_map[idx]);
        }
    }
    for (auto vertex : to_remove) {
        boost::clear_vertex(vertex, graph_);
        boost::remove_vertex(vertex, graph_);
    }
  }
} 
