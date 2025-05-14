#include <rclcpp/rclcpp.hpp>                          // ROS2 C++ 核心库
#include <sensor_msgs/msg/image.hpp>                  // 图像消息类型
#include <cv_bridge/cv_bridge.h>                      // ROS 与 OpenCV 图像转换桥梁
#include <opencv2/imgproc/imgproc.hpp>                // OpenCV 图像处理模块
#include <opencv2/highgui/highgui.hpp>                // OpenCV GUI 显示模块

// 定义一个共享的 ROS 节点指针
std::shared_ptr<rclcpp::Node> node;

// 图像回调函数，每当接收到一帧图像就会被调用
void CamRGBCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // 创建一个 cv_bridge 指针，用于将 ROS 图像消息转换为 OpenCV 格式
    cv_bridge::CvImagePtr cv_ptr;

    // 转换图像编码格式为 BGR8（OpenCV 常用格式）
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // 获取转换后的 OpenCV 图像
    cv::Mat imgOriginal = cv_ptr->image;

    // 在名为 "RGB" 的窗口中显示图像
    cv::imshow("RGB", imgOriginal);

    // 等待 1ms，处理 OpenCV 窗口事件，保证图像更新
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    // 创建一个名为 "cv_image_node" 的 ROS2 节点
    node = std::make_shared<rclcpp::Node>("cv_image_node");

    // 创建一个订阅者，订阅图像话题 "/kinect2/qhd/image_raw"
    // 消息队列大小为 1，回调函数为 CamRGBCallback
    auto rgb_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/kinect2/qhd/image_raw", 1, CamRGBCallback);

    // 创建一个名为 "RGB" 的 OpenCV 窗口
    cv::namedWindow("RGB");

    // 开始 ROS2 节点循环，等待和处理回调
    rclcpp::spin(node);

    // 当节点退出时，销毁所有 OpenCV 窗口
    cv::destroyAllWindows();

    // 关闭 ROS2
    rclcpp::shutdown();

    return 0;
}
