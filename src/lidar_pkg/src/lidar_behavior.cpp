#include <rclcpp/rclcpp.hpp>                         // ROS2核心功能头文件
#include <sensor_msgs/msg/laser_scan.hpp>            // LaserScan消息类型（激光雷达）
#include <geometry_msgs/msg/twist.hpp>               // Twist消息类型（用于控制机器人速度）

// 全局节点指针（用于日志输出等）
std::shared_ptr<rclcpp::Node> node;

// 全局速度控制发布器
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

// 控制机器人在避障之后延迟执行直行的计数器
int nCount = 0;

/**
 * @brief 激光雷达回调函数：读取前方距离并进行简单的避障逻辑
 * 
 * @param msg 接收到的LaserScan激光雷达消息
 */
void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // 获取激光雷达测量的距离数组长度
    int nNum = msg->ranges.size();

    // 取数组的中间值（一般代表机器人正前方）
    int nMid = nNum / 2;

    // 获取正前方的距离（单位：米）
    float fMidDist = msg->ranges[nMid];

    // 打印前方障碍物的距离
    RCLCPP_INFO(node->get_logger(), "ranges[%d] = %f m", nMid, fMidDist);

    // 如果当前处于避障倒计时状态，跳过处理（继续转弯）
    if (nCount > 0)
    {
        nCount--;  // 每次回调减少一次计数
        return;    // 不发布新的速度指令
    }

    // 创建一个速度控制消息对象
    geometry_msgs::msg::Twist vel_msg;

    // 如果正前方小于 1.5 米，则认为有障碍物
    if (fMidDist < 1.5f)
    {
        vel_msg.angular.z = 0.3;  // 设置机器人向左转（角速度0.3 rad/s）
        nCount = 100;             // 设置避障持续时间（例如 100 次回调周期）
    }
    else
    {
        vel_msg.linear.x = 0.1;   // 向前直行（线速度0.1 m/s）
    }

    // 发布速度控制指令
    vel_pub->publish(vel_msg);
}

/**
 * @brief 主函数：初始化ROS2系统，创建节点、订阅器与发布器，循环运行
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);  // 初始化ROS2

    // 创建节点对象，命名为 "lidar_behavior_node"
    node = std::make_shared<rclcpp::Node>("lidar_behavior_node");

    // 创建速度控制发布器，向 "/cmd_vel" 话题发布移动指令
    vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 创建订阅器，订阅激光雷达数据 "/scan"，回调函数为 LidarCallback
    auto lidar_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, LidarCallback);

    // 启动节点循环，接收并处理回调
    rclcpp::spin(node);

    // 程序退出前关闭 ROS2
    rclcpp::shutdown();

    return 0;
}
