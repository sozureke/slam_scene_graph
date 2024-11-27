#ifndef SG_SLAM_POINT_CLOUD_UTILS_HPP_
#define SG_SLAM_POINT_CLOUD_UTILS_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sg_slam/semantic_graph.hpp"

namespace sg_slam {

class PointCloudUtils : public rclcpp::Node {
public:
    PointCloudUtils(float z_min, float z_max, float voxel_leaf_size, float cluster_radius);
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // Делаем методы фильтрации публичными для использования в других файлах
    static pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloudByBounds(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float z_min, float z_max);
    static pcl::PointCloud<pcl::PointXYZ>::Ptr downsamplePointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float leaf_size);

private:
    float z_min_;
    float z_max_;
    float voxel_leaf_size_;
    float cluster_radius_;

    SemanticGraph semantic_graph_;
};

}  // namespace sg_slam

#endif  // SG_SLAM_POINT_CLOUD_UTILS_HPP_
ы