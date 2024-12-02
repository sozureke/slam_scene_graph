#ifndef SG_SLAM_SEMANTIC_GRAPH_HPP_
#define SG_SLAM_SEMANTIC_GRAPH_HPP_

#include <boost/graph/adjacency_list.hpp>
#include <string>
#include <utility>
#include <vector>
#include <unordered_set>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

namespace sg_slam {

struct NodeProperties {
    std::string object_type;
    std::pair<double, double> coordinates;
    std::pair<double, double> dimensions;
    std::string function;
    bool traversable;
    std::string access_status;
    int priority;
};

using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, NodeProperties>;
using Vertex = boost::graph_traits<Graph>::vertex_descriptor;

class SemanticGraph {
public:
    SemanticGraph();
    Vertex addNode(const NodeProperties& properties);
    void addEdge(Vertex node1, Vertex node2);
    void updateNodePosition(Vertex node, const std::pair<double, double>& new_coordinates);
    void removeOldNodes(const std::pair<double, double>& robot_position, double max_radius);
    void clusterNodes(double cluster_radius, int min_points_per_cluster);

    const Graph& getGraph() const;

private:
    Graph graph_;
};

} 

#endif
