#ifndef SG_SLAM_SEMANTIC_GRAPH_HPP_
#define SG_SLAM_SEMANTIC_GRAPH_HPP_

#include <boost/graph/adjacency_list.hpp>
#include <string>
#include <cmath>
#include <unordered_map>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> // Добавлено для работы с PCL

namespace sg_slam {

struct Position {
    double x;
    double y;
    double z;
};

struct ObjectDimensions {
    double width;
    double height;
    double length;

    ObjectDimensions(double w = 0.0, double h = 0.0, double l = 0.0);
};

struct NodeProperties {
    std::string object_type;
    Position coordinates;
    ObjectDimensions dimensions;
};

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, NodeProperties> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

class SemanticGraph {
public:
    SemanticGraph();
    Vertex addNode(const NodeProperties& properties);
    void addEdge(Vertex node1, Vertex node2);
    void updateNodePosition(Vertex node, const Position& new_coordinates);
    void removeOldNodes(const Position& robot_position, double max_radius);
    void clusterNodes(double cluster_radius, int min_points_per_cluster);
    void filterStableClusters(const std::vector<NodeProperties>& current_clusters);
    void publishStableClusters(); 
    void adaptiveClusterNodes(double base_cluster_radius, int min_points_per_cluster);
    double calculatePointDensity(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<int>& indices);
    void clearGraph();        

    const Graph& getGraph() const;

private:
    std::unordered_map<int, NodeProperties> stable_clusters;
    std::unordered_map<int, int> cluster_confirmations;
    Graph graph_;
};
} 

#endif
