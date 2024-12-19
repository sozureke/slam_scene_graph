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

void SemanticGraph::addRelationship(Vertex node1, Vertex node2, const std::string& relation) {
    auto [edge, success] = boost::add_edge(node1, node2, graph_);
    if (success) {
        graph_[edge].relation = relation;
        RCLCPP_INFO(rclcpp::get_logger("sg_slam"), "Relationship added between vertices: %s", relation.c_str());
    } else {
        RCLCPP_WARN(rclcpp::get_logger("sg_slam"), "Failed to add relationship between vertices.");
    }
}

const Graph& SemanticGraph::getGraph() const {
    return graph_;
}

Eigen::Vector3d SemanticGraph::calculateCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<int>& indices) const {
    Eigen::Vector3d centroid(0.0, 0.0, 0.0);
    for (const auto& idx : indices) {
        const auto& point = cloud->points[idx];
        centroid += Eigen::Vector3d(point.x, point.y, point.z);
    }
    centroid /= indices.size();
    return centroid;
}

Eigen::Matrix3d SemanticGraph::calculateOrientation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<int>& indices) const {
    Eigen::MatrixXd data(indices.size(), 3);
    for (size_t i = 0; i < indices.size(); ++i) {
        const auto& point = cloud->points[indices[i]];
        data.row(i) << point.x, point.y, point.z;
    }

    Eigen::Vector3d mean = data.colwise().mean();
    Eigen::MatrixXd centered = data.rowwise() - mean.transpose();
    Eigen::Matrix3d covariance = (centered.adjoint() * centered) / double(indices.size() - 1);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    
    return solver.eigenvectors();
}

void SemanticGraph::generateEdges() {
    double max_connection_distance = 10.0;
    double proximity_weight_base = 1.0;
    double adjacent_weight_base = 1.5;
    double attached_weight_base = 2.0;
    double parallel_weight_base = 2.5;
    double perpendicular_weight_base = 3.0;
    double close_weight_base = 3.5;
    double scale_factor_base = 1.5;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::unordered_map<int, Vertex> vertex_map;
    int index = 0;

    for (auto vertex : boost::make_iterator_range(boost::vertices(graph_))) {
        const auto& node = graph_[vertex];
        cloud->push_back(pcl::PointXYZ(node.coordinates.x, node.coordinates.y, node.coordinates.z));
        vertex_map[index++] = vertex;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); ++i) {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        pcl::PointXYZ searchPoint = cloud->points[i];

        if (kdtree.radiusSearch(searchPoint, max_connection_distance, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
                if (i == pointIdxRadiusSearch[j]) continue;

                auto vertex1 = vertex_map[i];
                auto vertex2 = vertex_map[pointIdxRadiusSearch[j]];

                const auto& node1 = graph_[vertex1];
                const auto& node2 = graph_[vertex2];

                double distance = std::sqrt(pointRadiusSquaredDistance[j]);

                double max_dimension = std::max({
                    node1.dimensions.width, node1.dimensions.height, node1.dimensions.length,
                    node2.dimensions.width, node2.dimensions.height, node2.dimensions.length
                });

                double dynamic_threshold = std::max(1.0, std::min(max_dimension * scale_factor_base, max_connection_distance));

                if (distance < dynamic_threshold) {
                    double weight = proximity_weight_base / distance;
                    addRelationship(vertex1, vertex2, "proximity (" + std::to_string(weight) + ")");
                }

                if ((node1.object_type == "Wall" || node1.object_type == "Unknown") &&
                    (node2.object_type == "Wall" || node2.object_type == "Unknown") &&
                    distance < dynamic_threshold * 1.5) {
                    double weight = adjacent_weight_base / distance;
                    addRelationship(vertex1, vertex2, "adjacent (" + std::to_string(weight) + ")");
                }

                if ((node1.object_type == "Door" && node2.object_type == "Wall") && distance < dynamic_threshold) {
                    double weight = attached_weight_base / distance;
                    addRelationship(vertex1, vertex2, "attached (" + std::to_string(weight) + ")");
                }

                if (std::abs(distance - max_dimension) < 1.0) {
                    double weight = close_weight_base / distance;
                    addRelationship(vertex1, vertex2, "close (" + std::to_string(weight) + ")");
                }

                Eigen::Vector3d dir1 = Eigen::Vector3d(node1.dimensions.width, node1.dimensions.height, node1.dimensions.length).normalized();
                Eigen::Vector3d dir2 = Eigen::Vector3d(node2.dimensions.width, node2.dimensions.height, node2.dimensions.length).normalized();
                double dot_product = dir1.dot(dir2);

                if (std::abs(dot_product - 1.0) < 0.1) {
                    double weight = parallel_weight_base / distance;
                    addRelationship(vertex1, vertex2, "parallel (" + std::to_string(weight) + ")");
                }

                if (std::abs(dot_product) < 0.1) {
                    double weight = perpendicular_weight_base / distance;
                    addRelationship(vertex1, vertex2, "perpendicular (" + std::to_string(weight) + ")");
                }
            }
        }
    }
}

double SemanticGraph::calculateDistance(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2) const {
    return (pos1 - pos2).norm();
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
}

void SemanticGraph::clusterNodes(double cluster_radius, int min_points_per_cluster) {
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

    if (cloud->empty()) {
        return;
    }

    graph_.clear();


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
