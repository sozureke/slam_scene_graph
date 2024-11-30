#include "sg_slam/semantic_graph.hpp"
#include <boost/graph/adjacency_list.hpp>
#include <cmath>
#include <unordered_set>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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
        return; 
    }

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
            continue; 
        }

        pcl::PointXYZ search_point = cloud->points[i];
        kd_tree.radiusSearch(search_point, cluster_radius, nearest_indices, nearest_distances);

        for (size_t j = 1; j < nearest_indices.size(); ++j) { 
            Vertex neighbor_vertex = vertex_map[nearest_indices[j]];
            if (to_remove.find(neighbor_vertex) == to_remove.end()) {
                to_remove.insert(neighbor_vertex);
            }
        }
    }

    for (auto vertex : to_remove) {
        boost::clear_vertex(vertex, graph_);
        boost::remove_vertex(vertex, graph_);
    }
}

} 
