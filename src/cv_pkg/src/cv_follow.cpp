#include <rclcpp/rclcpp.hpp>                           // ROS2 C++客户端库
#include <sensor_msgs/msg/image.hpp>                   // ROS2 图像消息类型
#include <cv_bridge/cv_bridge.h>                       // OpenCV与ROS图像消息转换桥
#include <opencv2/imgproc/imgproc.hpp>                 // OpenCV 图像处理模块
#include <opencv2/highgui/highgui.hpp>                 // OpenCV 图像显示窗口等
#include <geometry_msgs/msg/twist.hpp>                 // ROS2 控制小车的速度消息类型

// 声明 ROS2 节点的智能指针
std::shared_ptr<rclcpp::Node> node;

using namespace cv;
using namespace std;

// HSV颜色阈值，可通过滑块动态调节（此处写死）
static int iLowH = 10;
static int iHighH = 40;
static int iLowS = 90; 
static int iHighS = 255;
static int iLowV = 1;
static int iHighV = 255;

// 创建速度控制消息结构体和发布器
geometry_msgs::msg::Twist vel_cmd;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

// 摄像头RGB图像回调函数
void CamRGBCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // 将ROS图像消息转换为OpenCV格式（BGR8）
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    Mat imgOriginal = cv_ptr->image;

    // 转换颜色空间：BGR -> HSV
    Mat imgHSV;
    cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

    // 提高图像对比度：直方图均衡化V通道
    vector<Mat> hsvSplit;
    split(imgHSV, hsvSplit);                     // 分离H、S、V通道
    equalizeHist(hsvSplit[2], hsvSplit[2]);      // 直方图均衡化V通道
    merge(hsvSplit, imgHSV);                     // 合并通道回HSV图像

    // 根据HSV阈值二值化图像，得到目标区域
    Mat imgThresholded;
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

    // 图像形态学处理（先开运算去噪点，再闭运算填孔洞）
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
    morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

    // 寻找目标质心（白色区域的平均坐标）
    int nTargetX = 0;
    int nTargetY = 0;
    int nPixCount = 0;
    int nImgWidth = imgThresholded.cols;
    int nImgHeight = imgThresholded.rows;

    for (int y = 0; y < nImgHeight; y++) {
        for (int x = 0; x < nImgWidth; x++) {
            if (imgThresholded.data[y * nImgWidth + x] == 255) {
                nTargetX += x;
                nTargetY += y;
                nPixCount++;
            }
        }
    }

    if (nPixCount > 0) {
        // 计算目标质心
        nTargetX /= nPixCount;
        nTargetY /= nPixCount;
        printf("Target ( %d , %d )  PixelCount = %d\n", nTargetX, nTargetY, nPixCount);

        // 在RGB图像上绘制十字线表示目标中心
        Point line_begin = Point(nTargetX - 10, nTargetY);
        Point line_end = Point(nTargetX + 10, nTargetY);
        line(imgOriginal, line_begin, line_end, Scalar(255, 0, 0), 3);
        line_begin.x = nTargetX; line_begin.y = nTargetY - 10;
        line_end.x = nTargetX; line_end.y = nTargetY + 10;
        line(imgOriginal, line_begin, line_end, Scalar(255, 0, 0), 3);    

        // 计算控制命令：纵向前进速度与横向偏转角速度
        float fVelFoward = (nImgHeight / 2 - nTargetY) * 0.002;    // y轴偏差决定前进速度
        float fVelTurn = (nImgWidth / 2 - nTargetX) * 0.003;       // x轴偏差决定旋转速度

        vel_cmd.linear.x = fVelFoward;
        vel_cmd.linear.y = 0;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = fVelTurn;
    } else {
        // 没有检测到目标，停止运动
        printf("Target disappeared...\n");
        vel_cmd.linear.x = 0;
        vel_cmd.linear.y = 0;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = 0;
    }

    // 发布速度控制命令
    vel_pub->publish(vel_cmd);

    // 显示图像窗口
    imshow("Result", imgThresholded);
    imshow("RGB", imgOriginal);
    cv::waitKey(5);
}

int main(int argc, char **argv)
{
    // 初始化 ROS2 节点
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("cv_follow_node");

    // 创建速度话题发布器（控制/cmd_vel）
    vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 创建图像订阅器，订阅摄像头原始图像话题
    auto sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/kinect2/qhd/image_raw", 1, CamRGBCallback);

    // 创建显示窗口
    namedWindow("RGB");     
    namedWindow("Result"); 

    // 启动 ROS2 循环
    rclcpp::spin(node);

    // 清理资源
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
