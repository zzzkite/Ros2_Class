#include <rclcpp/rclcpp.hpp>                            // ROS2 核心库
#include <sensor_msgs/msg/image.hpp>                    // ROS 图像消息类型
#include <cv_bridge/cv_bridge.h>                        // ROS <-> OpenCV 图像桥
#include <opencv2/imgproc/imgproc.hpp>                  // OpenCV 图像处理
#include <opencv2/highgui/highgui.hpp>                  // OpenCV 显示窗口模块

// 创建全局节点指针（用于 ROS2 回调中访问）
std::shared_ptr<rclcpp::Node> node;

// 使用 OpenCV 和 std 命名空间
using namespace cv;
using namespace std;

// HSV 阈值参数（可由滑动条调节）
static int iLowH = 10;
static int iHighH = 40;
static int iLowS = 90; 
static int iHighS = 255;
static int iLowV = 1;
static int iHighV = 255;

// 回调函数：接收并处理摄像头图像
void CamRGBCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // 将 ROS 图像消息转换为 OpenCV 格式（BGR8）
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    Mat imgOriginal = cv_ptr->image;

    // 将 BGR 图像转换为 HSV
    Mat imgHSV;
    cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

    // 分离 HSV 通道，并对 V 通道进行直方图均衡化（提升亮度对比）
    vector<Mat> hsvSplit;
    split(imgHSV, hsvSplit);
    equalizeHist(hsvSplit[2], hsvSplit[2]);
    merge(hsvSplit, imgHSV);

    // 根据 HSV 范围进行二值化
    Mat imgThresholded;
    inRange(imgHSV,
            Scalar(iLowH, iLowS, iLowV),
            Scalar(iHighH, iHighS, iHighV),
            imgThresholded);

    // 使用形态学操作去噪（开运算+闭运算）
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
    morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

    // 遍历图像统计白色像素（目标）的质心位置
    int nTargetX = 0;
    int nTargetY = 0;
    int nPixCount = 0;
    int nImgWidth = imgThresholded.cols;
    int nImgHeight = imgThresholded.rows;
    for (int y = 0; y < nImgHeight; y++)
    {
        for (int x = 0; x < nImgWidth; x++)
        {
            if (imgThresholded.data[y * nImgWidth + x] == 255)
            {
                nTargetX += x;
                nTargetY += y;
                nPixCount++;
            }
        }
    }

    // 如果检测到目标，绘制十字标记中心点
    if (nPixCount > 0)
    {
        nTargetX /= nPixCount;
        nTargetY /= nPixCount;
        printf("Target (%d, %d) PixelCount = %d\n", nTargetX, nTargetY, nPixCount);

        // 水平线
        Point line_begin = Point(nTargetX - 10, nTargetY);
        Point line_end = Point(nTargetX + 10, nTargetY);
        line(imgOriginal, line_begin, line_end, Scalar(255, 0, 0));

        // 垂直线
        line_begin.x = nTargetX; line_begin.y = nTargetY - 10;
        line_end.x = nTargetX; line_end.y = nTargetY + 10;
        line(imgOriginal, line_begin, line_end, Scalar(255, 0, 0));
    }
    else
    {
        printf("Target disappeared...\n");
    }

    // 显示图像
    imshow("RGB", imgOriginal);       // 原始图像
    imshow("HSV", imgHSV);           // HSV 图像
    imshow("Result", imgThresholded); // 二值图像
    waitKey(5); // 等待 5 毫秒（用于 OpenCV 更新显示）
}

int main(int argc, char **argv)
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    // 创建 ROS2 节点
    node = std::make_shared<rclcpp::Node>("cv_hsv_node");

    // 创建图像订阅者，订阅话题 /kinect2/qhd/image_raw
    auto rgb_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/kinect2/qhd/image_raw", 1, CamRGBCallback);

    // 创建 HSV 阈值调节窗口与滑动条
    namedWindow("Threshold", WINDOW_AUTOSIZE);
    createTrackbar("LowH", "Threshold", &iLowH, 179);
    createTrackbar("HighH", "Threshold", &iHighH, 179);
    createTrackbar("LowS", "Threshold", &iLowS, 255);
    createTrackbar("HighS", "Threshold", &iHighS, 255);
    createTrackbar("LowV", "Threshold", &iLowV, 255);
    createTrackbar("HighV", "Threshold", &iHighV, 255);

    // 创建图像显示窗口
    namedWindow("RGB");
    namedWindow("HSV");
    namedWindow("Result");

    // 进入 ROS2 循环，等待回调执行
    rclcpp::spin(node);

    // 销毁所有 OpenCV 窗口
    destroyAllWindows();

    // 关闭 ROS2
    rclcpp::shutdown();
    return 0;
}
