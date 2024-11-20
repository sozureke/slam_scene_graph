#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

class JoyToCmdVel : public rclcpp::Node {
public:
    JoyToCmdVel() : Node("joy_to_cmd_vel") {
        this->declare_parameter("linear_sensitivity", 1.0);
        this->declare_parameter("angular_sensitivity", 1.0);
        this->get_parameter("linear_sensitivity", linear_sensitivity_);
        this->get_parameter("angular_sensitivity", angular_sensitivity_);

        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyToCmdVel::joyCallback, this, std::placeholders::_1)
        );

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        auto cmd_msg = geometry_msgs::msg::Twist();

        cmd_msg.linear.x = msg->axes[1] * linear_sensitivity_;
        cmd_msg.angular.z = msg->axes[0] * angular_sensitivity_;

        cmd_msg.linear.x = std::clamp(cmd_msg.linear.x, -max_linear_speed_, max_linear_speed_);
        cmd_msg.angular.z = std::clamp(cmd_msg.angular.z, -max_angular_speed_, max_angular_speed_);

        cmd_vel_publisher_->publish(cmd_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    double linear_sensitivity_ = 1.0;
    double angular_sensitivity_ = 1.0;
    const double max_linear_speed_ = 1.0;  // MAX LINEAR SPEED
    const double max_angular_speed_ = 1.0;  // MAX ANGLE SPEED
};