#ifndef SG_SLAM_SEMANTIC_GRAPH_HPP_
#define SG_SLAM_SEMANTIC_GRAPH_HPP_

#include <boost/graph/adjacency_list.hpp>
#include <string>
#include <unordered_map>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

struct EdgeProperties {
    std::string relation;
};

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, NodeProperties, EdgeProperties> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;

class SemanticGraph {
public:
    SemanticGraph();
    Vertex addNode(const NodeProperties& properties);
    void addEdge(Vertex node1, Vertex node2);
    void addRelationship(Vertex node1, Vertex node2, const std::string& relation);
    void updateNodePosition(Vertex node, const Position& new_coordinates);
    void removeOldNodes(const Position& robot_position, double max_radius);
    void clusterNodes(double cluster_radius, int min_points_per_cluster);
    void filterStableClusters(const std::vector<NodeProperties>& current_clusters);
    void adaptiveClusterNodes(double base_cluster_radius, int min_points_per_cluster);
    void generateEdges();
    double calculatePointDensity(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<int>& indices);
    std::string classifyObject(const NodeProperties& cluster);
    void clearGraph();
    const Graph& getGraph() const;

private:
    std::unordered_map<int, NodeProperties> stable_clusters;
    Graph graph_;
};

}

#endif
