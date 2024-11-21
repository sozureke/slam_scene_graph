#ifndef SG_SLAM_SEMANTIC_GRAPH_HPP_
#define SG_SLAM_SEMANTIC_GRAPH_HPP_

#include <boost/graph/adjacency_list.hpp>
#include <string>
#include <utility>
#include <nav_msgs/msg/occupancy_grid.hpp>

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
      void processMapData(const nav_msgs::msg::OccupancyGrid::SharedPtr& map);
      void removeOldNodes(const std::pair<double, double>& robot_position, double max_radius);
      void clusterNodes(double cluster_radius);

      const Graph& getGraph() const;

    private:
      Graph graph_;
  };

}

#endif
