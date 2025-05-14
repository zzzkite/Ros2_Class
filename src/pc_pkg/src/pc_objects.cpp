#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// 全局节点和TF监听器
std::shared_ptr<rclcpp::Node> node;
tf2_ros::Buffer::SharedPtr tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

// 点云回调函数
void PointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // 检查是否可以从传感器坐标系转换到 base_footprint
    bool result = tf_buffer_->canTransform(
        "base_footprint", msg->header.frame_id, msg->header.stamp);
    if (!result)
    {
        return;
    }

    // 执行点云坐标变换
    sensor_msgs::msg::PointCloud2 pc_footprint;
    pcl_ros::transformPointCloud("base_footprint", *msg, pc_footprint, *tf_buffer_);

    // 转换为 PCL 格式
    pcl::PointCloud<pcl::PointXYZ> cloud_src;
    pcl::fromROSMsg(pc_footprint, cloud_src);

    // --------------------- 直通滤波器过滤无效区域 ---------------------
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_src.makeShared());

    // X方向过滤 (前方 0.5 到 1.5 米)
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.5, 1.5);
    pass.filter(cloud_src);

    // Y方向过滤 (左右 -0.5 到 0.5 米)
    pass.setInputCloud(cloud_src.makeShared());
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.5, 0.5);
    pass.filter(cloud_src);

    // Z方向过滤 (高度 0.5 到 1.5 米)
    pass.setInputCloud(cloud_src.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.5, 1.5);
    pass.filter(cloud_src);

    // --------------------- 使用 RANSAC 拟合平面（例如桌面） ---------------------
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setInputCloud(cloud_src.makeShared());
    segmentation.setModelType(pcl::SACMODEL_PLANE); // 设置模型为平面
    segmentation.setMethodType(pcl::SAC_RANSAC);    // 使用RANSAC方法
    segmentation.setDistanceThreshold(0.05);        // 点到模型的距离阈值
    segmentation.setOptimizeCoefficients(true);
    segmentation.segment(*planeIndices, *coefficients);

    // 计算平面平均高度
    int point_num = planeIndices->indices.size();
    float points_z_sum = 0;
    for (int i = 0; i < point_num; i++)
    {
        int point_index = planeIndices->indices[i];
        points_z_sum += cloud_src.points[point_index].z;
    }
    float plane_height = points_z_sum / point_num;
    RCLCPP_INFO(node->get_logger(), "plane_height = %.2f", plane_height);

    // --------------------- 过滤平面以上的点云，准备聚类 ---------------------
    pass.setInputCloud(cloud_src.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(plane_height + 0.2, 1.5);
    pass.filter(cloud_src);

    // --------------------- 欧式聚类提取目标 ---------------------
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_src.makeShared());

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setInputCloud(cloud_src.makeShared());
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setClusterTolerance(0.1); // 两点之间最大距离
    ec.setSearchMethod(tree);

    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    // 输出聚类结果
    int object_num = cluster_indices.size();
    RCLCPP_INFO(node->get_logger(), "object_num = %d", object_num);

    for (int i = 0; i < object_num; i++)
    {
        int point_num = cluster_indices[i].indices.size();
        float points_x_sum = 0, points_y_sum = 0, points_z_sum = 0;

        for (int j = 0; j < point_num; j++)
        {
            int point_index = cluster_indices[i].indices[j];
            points_x_sum += cloud_src.points[point_index].x;
            points_y_sum += cloud_src.points[point_index].y;
            points_z_sum += cloud_src.points[point_index].z;
        }

        // 计算质心坐标
        float object_x = points_x_sum / point_num;
        float object_y = points_y_sum / point_num;
        float object_z = points_z_sum / point_num;

        RCLCPP_INFO(
            node->get_logger(),
            "object %d pos=(%.2f , %.2f , %.2f)", i, object_x, object_y, object_z);
    }

    RCLCPP_INFO(node->get_logger(), "---------------------");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 创建ROS节点
    node = std::make_shared<rclcpp::Node>("pointcloud_objects_node");

    // 初始化TF监听器
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 订阅点云话题
    auto pc_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/kinect2/sd/points", // 订阅 Kinect v2 的深度点云
        1,
        PointcloudCallback);

    rclcpp::spin(node);      // 进入ROS事件循环
    rclcpp::shutdown();      // 关闭
    return 0;
}
