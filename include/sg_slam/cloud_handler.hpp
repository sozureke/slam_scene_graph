#ifndef SG_SLAM_CLOUD_HANDLER_HPP_
#define SG_SLAM_CLOUD_HANDLER_HPP_

#include "sg_slam/semantic_graph.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace sg_slam {

class CloudHandler {
public:
    CloudHandler(SemanticGraph& graph, rclcpp::Node::SharedPtr node, double z_min, double z_max, double voxel_leaf_size, double cluster_radius);
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
    SemanticGraph& graph_;
    double z_min_, z_max_, voxel_leaf_size_, cluster_radius_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}

#endif 
