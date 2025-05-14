#include <rclcpp/rclcpp.hpp>                              // ROS2 C++ 核心库
#include <sensor_msgs/msg/image.hpp>                      // 图像消息类型
#include <sensor_msgs/msg/region_of_interest.hpp>         // 区域ROI消息类型，用于表示检测到的目标位置
#include <cv_bridge/cv_bridge.h>                          // OpenCV 与 ROS 图像转换桥
#include <opencv2/imgproc/imgproc.hpp>                    // OpenCV 图像处理模块
#include <opencv2/highgui/highgui.hpp>                    // OpenCV 图像显示模块

// 创建全局 ROS2 节点指针
std::shared_ptr<rclcpp::Node> node;

// 全局 OpenCV 图像变量，用于存储当前帧图像
cv::Mat imgFace;

// 创建图像发布器
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr frame_pub;

//
// 回调函数：接收摄像头图像
//
void CamRGBCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // 将 ROS 图像消息转换为 OpenCV 格式图像
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // 保存图像为全局变量，供后续人脸绘制使用
    imgFace = cv_ptr->image;

    // 将原始图像发布到“/face_detector_input”供人脸检测节点使用
    frame_pub->publish(*msg);
}

//
// 回调函数：接收人脸检测结果（ROI），并在图像中绘制红色矩形框
//
void FacePosCallback(const sensor_msgs::msg::RegionOfInterest::SharedPtr msg)
{
    // 绘制矩形框：左上角 (x_offset, y_offset)，右下角 (x+width, y+height)
    cv::rectangle(
        imgFace,
        cv::Point(msg->x_offset, msg->y_offset),
        cv::Point(msg->x_offset + msg->width, msg->y_offset + msg->height),
        cv::Scalar(0, 0, 255),     // 红色
        2,                         // 线宽为2像素
        cv::LINE_8                // 抗锯齿线条
    );

    // 显示图像
    cv::imshow("Face", imgFace);
    cv::waitKey(1);  // 刷新窗口
}

//
// 主函数
//
int main(int argc, char **argv)
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("cv_face_detect");

    // 订阅摄像头图像话题，获取原始图像
    auto rgb_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/kinect2/qhd/image_raw", 1, CamRGBCallback);

    // 发布图像，供人脸检测节点处理
    frame_pub = node->create_publisher<sensor_msgs::msg::Image>(
        "/face_detector_input", 1);

    // 订阅人脸检测结果（ROI），用于绘制矩形框
    auto face_sub = node->create_subscription<sensor_msgs::msg::RegionOfInterest>(
        "/face_position", 1, FacePosCallback);

    // 创建图像显示窗口
    cv::namedWindow("Face");

    // 开始ROS事件循环
    rclcpp::spin(node);

    // 程序结束，清理窗口与节点
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
