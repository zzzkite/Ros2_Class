#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

using namespace std::chrono_literals;

class TurtleCircleNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; //创建发布者的智能指针
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_; //订阅者的智能共享指针
    float target_x = 1.0;
    float target_y = 1.0;

public:
    explicit TurtleCircleNode(const std::string &node_name) : Node(node_name)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&TurtleCircleNode::on_pose_receive,this, std::placeholders::_1));
    }

    void on_pose_receive(const turtlesim::msg::Pose::SharedPtr pose)
    {
        auto msg = geometry_msgs::msg::Twist();
        auto current_x = pose->x;
        auto current_y = pose->y;
        RCLCPP_INFO(get_logger(),"当前x:%f: 当前y:%f", current_x, current_y);

        auto distance = std::sqrt((target_x-current_x)*(target_x-current_x) + (target_y-current_y)*(target_y-current_y));
        auto angle = std::atan2((target_y-current_y), (target_x-current_x));

        if(fabs(angle - pose->theta)>0.1)msg.angular.z = fabs(angle - pose->theta);
        msg.linear.x = distance;

        publisher_->publish(msg);
    }

};

int main(int argc, char*argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TurtleCircleNode>("turtle_control");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}