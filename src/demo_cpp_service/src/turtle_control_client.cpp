#include "rclcpp/rclcpp.hpp"
#include "service_interface/srv/patrol.hpp"
#include "chrono"
#include <ctime>

// 定义Patrol服务的别名
using Patrol = service_interface::srv::Patrol;
using namespace std::chrono_literals;

// 创建PatrolClientNode类，继承自rclcpp::Node
class PatrolClientNode : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_; // 定时器，用于周期性发送请求
    rclcpp::Client<Patrol>::SharedPtr client_; // 客户端对象

public:
    // 构造函数，显式声明，防止隐式转换
    explicit PatrolClientNode(const std::string &node_name) : Node(node_name)
    {
        // 创建服务客户端，连接"turtle_patrol"服务
        client_ = this->create_client<Patrol>("turtle_patrol");
        
        // 创建定时器，每7秒执行一次请求
        timer_ = this->create_wall_timer(7s, [&]() -> void {
            // 1. 检查服务是否可用
            while (!this->client_->wait_for_service(1s)) 
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "等待服务器上线过程中rclcpp挂了");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "等待服务器上线中...");
                srand(time(NULL)); // 设定随机种子
            }

            // 2. 创建请求对象
            auto request = std::make_shared<Patrol::Request>();
            request->target_x = rand() % 15; // 随机生成目标x坐标（0-14）
            request->target_y = rand() % 15; // 随机生成目标y坐标（0-14）
            RCLCPP_INFO(this->get_logger(), "准备好点 %f , %f", request->target_x, request->target_y);

            // 3. 发送异步请求
            this->client_->async_send_request(request, [&](rclcpp::Client<Patrol>::SharedFuture result_future) -> void {
                auto response = result_future.get();
                // 这里原代码有错误，应该使用 "==" 进行比较，而不是 "="
                if (response->result == response->SUCCESS) 
                {
                    RCLCPP_INFO(this->get_logger(), "请求获取巡逻目标点成功");
                }
                else if (response->result == response->FAIL) 
                {
                    RCLCPP_INFO(this->get_logger(), "请求获取巡逻目标点失败");
                }
            });
        });
    }
};

// 主函数
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv); // 初始化ROS2
    auto node = std::make_shared<PatrolClientNode>("turtle_control_client"); // 创建客户端节点实例
    rclcpp::spin(node); // 让节点进入事件循环
    rclcpp::shutdown(); // 关闭ROS2
    return 0;
}
