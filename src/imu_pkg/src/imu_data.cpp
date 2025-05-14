#include <rclcpp/rclcpp.hpp>                    // ROS2 核心库
#include <sensor_msgs/msg/imu.hpp>              // IMU 消息类型
#include <tf2/LinearMath/Quaternion.h>          // tf2 中的四元数类
#include <tf2/LinearMath/Matrix3x3.h>           // 用于四元数转欧拉角的矩阵类

// 定义全局节点指针
std::shared_ptr<rclcpp::Node> node;

// 回调函数：每次收到 /imu 话题的数据时调用
void IMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // 从消息中提取四元数
    tf2::Quaternion tf2_quaternion;
    tf2_quaternion.setX(msg->orientation.x);
    tf2_quaternion.setY(msg->orientation.y);
    tf2_quaternion.setZ(msg->orientation.z);
    tf2_quaternion.setW(msg->orientation.w);

    // 将四元数转换为旋转矩阵
    tf2::Matrix3x3 matrix(tf2_quaternion);

    // 从旋转矩阵中提取欧拉角（单位为弧度）
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);  // 按 Roll-Pitch-Yaw 顺序提取

    // 将弧度转换为角度（度数）
    roll = roll * 180 / M_PI;
    pitch = pitch * 180 / M_PI;
    yaw = yaw * 180 / M_PI;

    // 输出欧拉角到终端
    RCLCPP_INFO(node->get_logger(), "roll= %.0f pitch= %.0f yaw= %.0f", roll, pitch, yaw);
}

int main(int argc, char** argv)
{
    // 初始化 ROS2 节点
    rclcpp::init(argc, argv);

    // 创建名为 imu_data_node 的节点
    node = std::make_shared<rclcpp::Node>("imu_data_node");

    // 创建订阅器，订阅话题 /imu，队列长度为 10，回调函数为 IMUCallback
    auto sub = node->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, IMUCallback);

    // 进入 ROS2 事件循环，等待消息
    rclcpp::spin(node);

    // 程序结束时关闭 ROS2
    rclcpp::shutdown();

    return 0;
}
