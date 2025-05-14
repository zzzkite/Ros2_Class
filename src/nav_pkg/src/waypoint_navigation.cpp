#include <rclcpp/rclcpp.hpp>              // ROS2核心库
#include <std_msgs/msg/string.hpp>        // 字符串消息类型

// 定义一个全局的节点指针，供回调函数中使用
std::shared_ptr<rclcpp::Node> node;

/**
 * @brief 回调函数：处理导航结果消息
 * @param msg 接收到的导航结果消息（字符串）
 */
void ResultCallback(const std_msgs::msg::String::SharedPtr msg)
{
    // 如果接收到的消息内容是 "navi done"，说明已到达导航点
    if (msg->data == "navi done")
    {
        RCLCPP_INFO(node->get_logger(), "Arrived !");  // 在终端打印提示信息
    }
}

int main(int argc, char** argv)
{
    // 初始化 ROS2 系统
    rclcpp::init(argc, argv);

    // 创建节点对象，节点名称为 "waypoint_navigation_node"
    node = std::make_shared<rclcpp::Node>("waypoint_navigation_node");

    // 创建一个发布器：发布导航点编号至 "/waterplus/navi_waypoint"
    auto navigation_pub = node->create_publisher<std_msgs::msg::String>(
        "/waterplus/navi_waypoint", 10);  // 队列大小为10

    // 创建一个订阅器：订阅 "/waterplus/navi_result"，监听导航是否完成
    auto result_sub = node->create_subscription<std_msgs::msg::String>(
        "/waterplus/navi_result", 10, ResultCallback);  // 回调函数为ResultCallback

    // 稍作延迟，等待系统初始化（确保订阅器建立成功）
    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    // 构造导航目标点消息（此处为导航点编号 "1"）
    std_msgs::msg::String waypoint_msg;
    waypoint_msg.data = "1";

    // 发布导航点编号，通知机器人前往目标
    navigation_pub->publish(waypoint_msg);

    // 启动节点的事件循环，监听回调
    rclcpp::spin(node);

    // 退出时释放资源
    rclcpp::shutdown();
    return 0;
}
