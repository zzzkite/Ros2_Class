#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

geometry_msgs::msg::Twist vel_msg;


void set_vel(geometry_msgs::msg::Twist &vel_msg)
{
    vel_msg.linear.x = 0.1;
    vel_msg.linear.y = 0.0;
    vel_msg.linear.z = 0.0;
    vel_msg.angular.x = 0.0;
    vel_msg.angular.y = 0.0;
    vel_msg.angular.z = 0.0;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    set_vel(vel_msg);
    auto node = std::make_shared<rclcpp::Node>("velocity_command_node");
    auto vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    rclcpp::Rate look_rate(1); // 设定休眠周期为1Hz(即确定发送周期)
    while (rclcpp::ok())
    {
       vel_pub->publish(vel_msg);
       look_rate.sleep();
    }
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}