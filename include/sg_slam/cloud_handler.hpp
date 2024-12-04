#ifndef SG_SLAM_CLOUD_HANDLER_HPP_
#define SG_SLAM_CLOUD_HANDLER_HPP_

#include "sg_slam/semantic_graph.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <memory>
#include <mutex>

namespace sg_slam {

class CloudHandler {
public:
    CloudHandler(SemanticGraph& graph, double cluster_radius, int min_points_per_cluster, std::mutex& graph_mutex);
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void setClusterRadius(double radius);
    void setMinPointsPerCluster(int points);

private:
    SemanticGraph& graph_;
    double cluster_radius_;
    int min_points_per_cluster_;
    std::mutex& graph_mutex_;
};

}

#endif