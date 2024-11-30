#include "sg_slam/cloud_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace sg_slam {

CloudHandler::CloudHandler(SemanticGraph& graph) : graph_(graph) {}

void CloudHandler::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received cloud data.");

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (*iter_z > 0.0 && *iter_z < 5.0) {
            NodeProperties props;
            props.object_type = "PointCloud";
            props.coordinates = {*iter_x, *iter_y};
            graph_.addNode(props);
        }
    }
    graph_.clusterNodes(1.0); 
}

} 
