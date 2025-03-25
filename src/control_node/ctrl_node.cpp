#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <chrono>

using namespace std;

void setSpeed(geometry_msgs::msg::Twist & _speed_msg,const double _lx,const double _ly,const double _az){
    _speed_msg.linear.x = _lx;
    _speed_msg.linear.y = _ly;
    _speed_msg.linear.z = 0.0;

    _speed_msg.angular.x = 0.0;
    _speed_msg.angular.y = 0.0;
    _speed_msg.angular.z = _az;

}

int main(int argc,char * argv[]){

    rclcpp::init(argc,argv);

    auto node = make_shared<rclcpp::Node>("control_node") ;
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    
    geometry_msgs::msg::Twist speed_msg;

    rclcpp::Rate spinRate(10);

    enum State { STRAIGHT, TURNING };
    State currentState = STRAIGHT;
    rclcpp::Time phaseStartTime = node->get_clock()->now();
    const double straightDuration = 3.0; // 直线持续时间(s)
    const double turnDuration = 6.0;     // 转弯持续时间(s) (对应2π rad/s角速度)
    
    RCLCPP_INFO(node->get_logger(), "===== 运动开始 =====");

    while (rclcpp::ok()) {
        // 计算阶段耗时
        double elapsed = node->get_clock()->now().seconds() - phaseStartTime.seconds();

        switch(currentState) {
            case STRAIGHT: // 直线运动
                if (elapsed >= straightDuration) {
                    RCLCPP_INFO(node->get_logger(), "===== 直线完成，开始转弯 =====");
                    currentState = TURNING;
                    phaseStartTime = node->get_clock()->now();
                } else {
                    // 设置直线速度：x=0.5 m/s，z=0
                    setSpeed(speed_msg, 0.5, 0.0, 0.0);
                }
                break;
                
            case TURNING: // 圆周运动
                if (elapsed >= turnDuration) {
                    RCLCPP_INFO(node->get_logger(), "===== 转弯完成，停止运动 =====");
                    setSpeed(speed_msg, 0.0, 0.0, 0.0); // 停止
                } else {
                    // 设置圆周运动：v=0.3 m/s，ω=0.5 rad/s → 半径r=0.6m
                    setSpeed(speed_msg, 0.3, 0.0, 0.5);
                }
                break;
        }

        pub->publish(speed_msg); // 发布速度指令
        spinRate.sleep();         // 维持10Hz频率
    }

    rclcpp::shutdown();

    return 0;
}