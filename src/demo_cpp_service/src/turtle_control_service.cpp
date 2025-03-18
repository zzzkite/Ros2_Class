#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "service_interface/srv/patrol.hpp"

// 定义Patrol服务的别名
using Patrol = service_interface::srv::Patrol;
using namespace std::chrono_literals;

// 创建TurtleCircleNode类，继承自rclcpp::Node
class TurtleCircleNode : public rclcpp::Node
{
private:
    // 创建一个发布者，用于发布小乌龟的速度控制指令
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    // 创建一个订阅者，用于订阅小乌龟的位置信息
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
    // 创建一个服务，用于接收目标坐标
    rclcpp::Service<Patrol>::SharedPtr service_;
    // 目标坐标，默认为(1.0, 1.0)
    float target_x = 1.0;
    float target_y = 1.0;

public:
    // 构造函数，显式声明，防止隐式转换
    explicit TurtleCircleNode(const std::string &node_name) : Node(node_name)
    {
        // 创建服务，名为"turtle_patrol"，用于接收目标坐标请求
        service_ = this->create_service<Patrol>("turtle_patrol", 
            [&](const Patrol::Request::SharedPtr request, Patrol::Response::SharedPtr response) -> void {
                // 检查目标坐标是否在合法范围内（1.0 <= x, y <= 12.0）
                if (request->target_x > 0 && request->target_x <= 12.0f && 
                    request->target_y > 0 && request->target_y <= 12.0f)
                {
                    this->target_x = request->target_x;
                    this->target_y = request->target_y;
                    response->result = response->SUCCESS; // 成功
                }
                else
                {
                    response->result = response->FAIL; // 失败
                }
            });

        // 创建发布者，向"/turtle1/cmd_vel"话题发布Twist消息（线速度和角速度）
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        
        // 创建订阅者，订阅"/turtle1/pose"话题，获取乌龟的当前位置
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, 
            std::bind(&TurtleCircleNode::on_pose_receive, this, std::placeholders::_1));
    }

    // 处理订阅消息的回调函数
    void on_pose_receive(const turtlesim::msg::Pose::SharedPtr pose)
    {
        auto msg = geometry_msgs::msg::Twist();
        float current_x = pose->x;
        float current_y = pose->y;
        RCLCPP_INFO(get_logger(), "当前x:%f, 当前y:%f", current_x, current_y);
        
        // 计算当前位置与目标位置的距离
        float distance = std::sqrt((target_x - current_x) * (target_x - current_x) +
                                   (target_y - current_y) * (target_y - current_y));
        // 计算朝向目标点的角度
        float angle = std::atan2((target_y - current_y), (target_x - current_x));
        
        // 角度调整：如果偏差较大，先旋转
        if (fabs(angle - pose->theta) > 0.2)
        {
            msg.angular.z = fabs(angle - pose->theta);
        }
        else // 否则前进
        {
            msg.linear.x = distance;
        }
        
        // 发布速度控制指令
        publisher_->publish(msg);
    }
};

// 主函数
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv); // 初始化ROS2
    auto node = std::make_shared<TurtleCircleNode>("turtle_control"); // 创建节点实例
    rclcpp::spin(node); // 让节点进入事件循环
    rclcpp::shutdown(); // 关闭ROS2
    return 0;
}
