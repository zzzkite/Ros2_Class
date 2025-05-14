#include <rclcpp/rclcpp.hpp>                         // ROS 2 C++ 核心功能头文件
#include <sensor_msgs/msg/laser_scan.hpp>            // 激光雷达 LaserScan 消息类型

// 创建一个全局的 ROS2 节点指针（用于日志输出等）
std::shared_ptr<rclcpp::Node> node;

/**
 * @brief 激光雷达数据回调函数
 *        每次接收到 /scan 激光数据时自动调用
 *
 * @param msg 接收到的 LaserScan 消息指针
 */
void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // 获取雷达扫描数据的总点数（比如360度激光就是360个点）
    int nNum = msg->ranges.size();    

    // 计算中间位置的索引值（正前方）
    int nMid = nNum / 2;

    // 获取正前方激光测得的距离值（单位：米）
    float fMidDist = msg->ranges[nMid];

    // 打印正前方的距离信息
    RCLCPP_INFO(node->get_logger(), "ranges[%d] = %f m", nMid, fMidDist);
}

/**
 * @brief 主函数 —— 初始化节点，订阅激光雷达话题，循环运行
 */
int main(int argc, char** argv)
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    // 创建节点对象，名称为 lidar_data_node
    node = std::make_shared<rclcpp::Node>("lidar_data_node");

    // 创建订阅者，订阅 "/scan" 话题（通常是激光雷达输出话题）
    // 队列长度为10，绑定回调函数 LidarCallback
    auto lidar_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, LidarCallback);

    // 循环运行节点，处理回调函数
    rclcpp::spin(node);

    // 程序退出前清理资源
    rclcpp::shutdown();

    return 0;
}
