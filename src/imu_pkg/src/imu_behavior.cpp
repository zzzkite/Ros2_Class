#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>                 // IMU消息类型
#include <geometry_msgs/msg/twist.hpp>             // 用于发布速度指令
#include <tf2/LinearMath/Quaternion.h>             // 四元数库
#include <tf2/LinearMath/Matrix3x3.h>              // 四元数转RPY的工具类

// 创建一个全局节点指针
std::shared_ptr<rclcpp::Node> node;

// 定义一个发布器，用于发布速度消息
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

/**
 * @brief IMU 回调函数 —— 处理 IMU 数据并生成速度指令
 */
void IMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // 将 ROS 中的 orientation 转为 tf2 的四元数对象
    tf2::Quaternion tf2_quaternion;
    tf2_quaternion.setX(msg->orientation.x);
    tf2_quaternion.setY(msg->orientation.y);
    tf2_quaternion.setZ(msg->orientation.z);
    tf2_quaternion.setW(msg->orientation.w);

    // 将四元数转换为旋转矩阵，从中提取 Roll、Pitch、Yaw
    tf2::Matrix3x3 matrix(tf2_quaternion);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);  // 单位为弧度

    // 将角度转换为度（方便理解）
    roll = roll * 180 / M_PI;
    pitch = pitch * 180 / M_PI;
    yaw = yaw * 180 / M_PI;

    // 打印当前姿态角
    RCLCPP_INFO(node->get_logger(), "roll= %.0f pitch= %.0f yaw= %.0f", roll, pitch, yaw);

    // 目标朝向角度（目标yaw角）为 90°
    double target_yaw = 90;

    // 计算当前yaw与目标yaw之间的差值（角度）
    double diff_angle = target_yaw - yaw;

    // 构建速度消息，根据 yaw 的差值控制转向
    geometry_msgs::msg::Twist vel_msg;
    vel_msg.angular.z = diff_angle * 0.01;  // 差值越大，转得越快（比例控制）
    vel_msg.linear.x = 0.1;                 // 前进速度固定为 0.1 m/s

    // 发布速度指令
    vel_pub->publish(vel_msg);
}

/**
 * @brief 主函数 —— 初始化 ROS 节点，订阅 IMU 数据，发布速度指令
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);  // 初始化ROS2系统

    // 创建节点
    node = rclcpp::Node::make_shared("imu_behavior_node");

    // 创建 IMU 订阅者（订阅话题名可能需要根据实际情况改为 "/imu/data"）
    auto sub = node->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, IMUCallback);  // 注意：此处的话题名 "imu" 可改为实际话题名

    // 创建速度控制发布器，发布至 "/cmd_vel" 控制移动
    vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 节点开始循环接收消息
    rclcpp::spin(node);

    // 程序退出时释放资源
    rclcpp::shutdown();
    return 0;
}
