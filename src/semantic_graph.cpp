#include "sg_slam/semantic_graph.hpp"
#include <boost/graph/adjacency_list.hpp>
#include <cmath>
#include <unordered_set>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>

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

void SemanticGraph::clusterNodes(double cluster_radius, int min_points_per_cluster) {
    if (boost::num_vertices(graph_) < 2) {
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::unordered_map<int, Vertex> vertex_map;
    int index = 0;
    for (auto vertex : boost::make_iterator_range(boost::vertices(graph_))) {
        pcl::PointXYZ point(graph_[vertex].coordinates.first, graph_[vertex].coordinates.second, 0.0);
        cloud->push_back(point);
        vertex_map[index++] = vertex;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
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
            Vertex vertex = vertex_map[idx];
            to_remove.insert(vertex);
        }
    }
    for (auto vertex : to_remove) {
        boost::clear_vertex(vertex, graph_);
        boost::remove_vertex(vertex, graph_);
    }
}

} 