#include <rclcpp/rclcpp.hpp>                            // ROS2 C++核心接口
#include <sensor_msgs/msg/point_cloud2.hpp>             // ROS2 点云消息格式（PointCloud2）
#include <pcl/point_types.h>                            // PCL 点类型定义（如 pcl::PointXYZ）
#include <pcl/point_cloud.h>                            // PCL 点云数据结构
#include <pcl_conversions/pcl_conversions.h>            // ROS 与 PCL 点云之间的转换工具

// 创建全局 ROS2 节点指针（为了在回调中使用日志打印）
std::shared_ptr<rclcpp::Node> node;

/**
 * @brief 点云回调函数：接收来自 Kinect 的点云数据并打印每个点的坐标。
 *
 * @param msg 接收到的 ROS2 PointCloud2 消息
 */
void PointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // 创建 PCL 格式的点云对象（类型为 PointXYZ：包含 x, y, z 三个坐标值）
  pcl::PointCloud<pcl::PointXYZ> pointCloudIn;

  // 将 ROS 的 PointCloud2 消息转换为 PCL 点云对象
  pcl::fromROSMsg(*msg, pointCloudIn);

  // 获取点云中点的数量
  int cloudSize = pointCloudIn.points.size();

  // 遍历每一个点，输出其三维坐标
  for (int i = 0; i < cloudSize; i++)
  {
    RCLCPP_INFO(node->get_logger(),
                "[i= %d] ( %.2f , %.2f , %.2f)",
                i,
                pointCloudIn.points[i].x,
                pointCloudIn.points[i].y,
                pointCloudIn.points[i].z);
  }
}

/**
 * @brief 主函数：初始化 ROS2 节点，订阅点云数据，并进入回调循环
 */
int main(int argc, char **argv)
{
  // 初始化 ROS2
  rclcpp::init(argc, argv);

  // 创建 ROS2 节点，命名为 "pointcloud_data_node"
  node = std::make_shared<rclcpp::Node>("pointcloud_data_node");

  // 创建订阅器：订阅 Kinect 的点云数据话题 "/kinect2/sd/points"
  auto pc_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/kinect2/sd/points",       // 点云话题名
    1,                          // 队列长度
    PointcloudCallback          // 回调函数
  );

  // 启动 ROS2 事件循环（等待并处理消息）
  rclcpp::spin(node);

  // 节点关闭后执行清理
  rclcpp::shutdown();
  return 0;
}
