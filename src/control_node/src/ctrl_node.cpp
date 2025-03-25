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
    const double straightDuration = 3.0; 
    const double turnDuration = 6.0;     
    
    RCLCPP_INFO(node->get_logger(), "===== 运动开始 =====");

    while (rclcpp::ok()) {
        // 计时
        double elapsed = node->get_clock()->now().seconds() - phaseStartTime.seconds();

        switch(currentState) {
            case STRAIGHT: // 直线
                if (elapsed >= straightDuration) {
                    RCLCPP_INFO(node->get_logger(), "===== 直线完成，开始转弯 =====");
                    currentState = TURNING;
                    phaseStartTime = node->get_clock()->now();
                } else {
                    setSpeed(speed_msg, 0.5, 0.0, 0.0);
                }
                break;
                
            case TURNING: // 转弯
                if (elapsed >= turnDuration) {
                    RCLCPP_INFO(node->get_logger(), "===== 转弯完成，停止运动 =====");
                    setSpeed(speed_msg, 0.0, 0.0, 0.0); 
                } else {
                    setSpeed(speed_msg, 0.3, 0.0, 0.5);
                }
                break;
        }

        pub->publish(speed_msg); 
        spinRate.sleep();         
    }

    rclcpp::shutdown();

    return 0;
}