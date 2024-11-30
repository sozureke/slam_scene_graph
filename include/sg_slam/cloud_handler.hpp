#ifndef SG_SLAM_CLOUD_HANDLER_HPP_
#define SG_SLAM_CLOUD_HANDLER_HPP_

#include "sg_slam/semantic_graph.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unordered_set>

namespace sg_slam {

class CloudHandler {
public:
    CloudHandler(SemanticGraph& graph);
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
    SemanticGraph& graph_;
};

}

#endif 
