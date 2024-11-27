#include "sg_slam/cloud_handler.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

namespace sg_slam {

CloudHandler::CloudHandler(SemanticGraph& graph, rclcpp::Node::SharedPtr node, double z_min, double z_max, double voxel_leaf_size, double cluster_radius)
    : graph_(graph), z_min_(z_min), z_max_(z_max), voxel_leaf_size_(voxel_leaf_size), cluster_radius_(cluster_radius) {
    subscription_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lio_sam/mapping/cloud_registered", 10, std::bind(&CloudHandler::cloudCallback, this, std::placeholders::_1));
}

void CloudHandler::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min_, z_max_);
    pass.filter(*cloud);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_grid.filter(*cloud);

    for (const auto& point : cloud->points) {
        NodeProperties props;
        props.coordinates = {point.x, point.y};
        props.object_type = "PointCloud";
        graph_.addNode(props);
    }

    graph_.clusterNodes(cluster_radius_);
}

}  // namespace sg_slam
