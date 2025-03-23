#include "rclcomm.h"

#define REFRESH_RATE 50

rclcomm::rclcomm()
{
    int argc = 0;
    char **argv = NULL;
    rclcpp::init(argc, argv);
    m_executor = new rclcpp::executors::MultiThreadedExecutor;
    node = rclcpp::Node::make_shared("OriginBot");
    m_executor->add_node(node);

    // 创建多线程sub1_obt、sub_laser_obt
    callback_group_laser = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_other = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub1_obt = rclcpp::SubscriptionOptions();
    sub1_obt.callback_group = callback_group_other;

    auto sub_laser_obt = rclcpp::SubscriptionOptions();
    sub_laser_obt.callback_group = callback_group_laser;

    _navPosePublisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    _initPosePublisher = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
    _publisher = node->create_publisher<std_msgs::msg::Int32>("ros2_qt_dmeo_publish", 10);
    CMD_VEL_publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 5);
    m_map_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        std::bind(&rclcomm::map_callback, this, std::placeholders::_1), sub1_obt);
    m_localCostMapSub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/local_costmap/costmap", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        std::bind(&rclcomm::localCostMapCallback, this, std::placeholders::_1), sub1_obt);
    m_globalCostMapSub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        std::bind(&rclcomm::globalCostMapCallback, this, std::placeholders::_1), sub1_obt);
    m_laser_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 20, std::bind(&rclcomm::laser_callback, this, std::placeholders::_1), sub_laser_obt);
    m_path_sub = node->create_subscription<nav_msgs::msg::Path>(
        "/plan", 20, std::bind(&rclcomm::path_callback, this, std::placeholders::_1), sub1_obt);
    _local_path_sub = node->create_subscription<nav_msgs::msg::Path>(
        "/local_plan", 20, std::bind(&rclcomm::local_path_callback, this, std::placeholders::_1), sub1_obt);
    m_odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 20, std::bind(&rclcomm::odom_callback, this, std::placeholders::_1), sub1_obt);

    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    m_transform_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
}

void rclcomm::run()
{
    rclcpp::WallRate loop_rate(REFRESH_RATE);
    while (rclcpp::ok())
    {
        m_executor->spin_some();
        getRobotPose();
        loop_rate.sleep();
    }
    rclcpp::shutdown();
}

void rclcomm::getRobotPose()
{
    try
    {
        geometry_msgs::msg::TransformStamped transform =
            m_tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
        geometry_msgs::msg::Quaternion msg_quat = transform.transform.rotation;
        tf2::Quaternion q;
        tf2::fromMsg(msg_quat, q);
        tf2::Matrix3x3 mat(q);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        double x = transform.transform.translation.x;
        double y = transform.transform.translation.y;
        QPointF trans_pose = transWordPoint2Scene(QPointF(x, y));
        m_currPose.x = trans_pose.x();
        m_currPose.y = trans_pose.y();
        m_currPose.theta = yaw;
        emit emitUpdateRobotPose(m_currPose);
    }
    catch (tf2::TransformException &ex)
    {
        qDebug() << "map  transform error:" << ex.what();
    }
}

void rclcomm::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    geometry_msgs::msg::Quaternion msg_quat = msg->pose.pose.orientation;
    RobotState state;
    tf2::Quaternion q;
    tf2::Matrix3x3 mat(q);
    double roll, pitch, yaw;

    tf2::fromMsg(msg_quat, q);
    mat.getRPY(roll, pitch, yaw);
    state.vx = (double)msg->twist.twist.linear.x;
    state.vy = (double)msg->twist.twist.linear.y;
    state.w = (double)msg->twist.twist.angular.z;
    state.x = (double)msg->pose.pose.position.x;
    state.y = (double)msg->pose.pose.position.y;
    state.theta = yaw;

    emit emitOdomInfo(state);
}
void rclcomm::local_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    try
    {
        geometry_msgs::msg::PointStamped point_map_frame;
        geometry_msgs::msg::PointStamped point_odom_frame;
        QPolygonF emit_points;
        for (unsigned int i = 0; i < msg->poses.size(); i++)
        {
            double x = msg->poses.at(i).pose.position.x;
            double y = msg->poses.at(i).pose.position.y;
            point_odom_frame.point.x = x;
            point_odom_frame.point.y = y;
            point_odom_frame.header.frame_id = msg->header.frame_id;
            m_tf_buffer->transform(point_odom_frame, point_map_frame, "map");
            QPointF point;
            point.setX(point_map_frame.point.x);
            point.setY(point_map_frame.point.y);
            point = transWordPoint2Scene(point);
            emit_points.push_back(point);
        }
        emit emitUpdateLocalPath(emit_points);
    }
    catch (tf2::TransformException &ex)
    {
        qDebug() << "local path transform error:" << ex.what();
    }
}

QImage rclcomm::rotateMapWithY(QImage map)
{
    QImage res = map;
    for (int x = 0; x < map.width(); x++)
    {
        for (int y = 0; y < map.height(); y++)
        {
            res.setPixelColor(x, map.height() - y - 1, map.pixelColor(x, y));
        }
    }
    return res;
}
QPointF rclcomm::transWordPoint2Scene(QPointF point)
{
    QPointF ret;
    ret.setX(m_wordOrigin.x() + point.x() / m_resolution);
    ret.setY(m_wordOrigin.y() - point.y() / m_resolution);
    return ret;
}
QPointF rclcomm::transScenePoint2Word(QPointF point)
{
    QPointF ret;
    ret.setX((point.x() - m_wordOrigin.x()) * m_resolution);
    ret.setY(-1 * (point.y() - m_wordOrigin.y()) * m_resolution);
    return ret;
}

void rclcomm::GETHORIZONTALSLIADER_LINEAR(int linear)
{
    qlinear = linear;
    qDebug() << "Linear" << qlinear;
}
void rclcomm::GETHORIZONTALSLIADER_RAW(int raw)
{
    qraw = raw;
    qDebug() << "RAW" << qraw;
}

void rclcomm::slot_cmd_control()
{
    QPushButton *btn = qobject_cast<QPushButton *>(sender());
    char key = btn->text().toStdString()[0];
    qDebug() << "key" << key;
    auto message = geometry_msgs::msg::Twist();
    if (key == 'i')
    {
        message.linear.x = 0.05 * qlinear;
        message.angular.z = 0.0 * qraw;
    }
    else if (key == ',')
    {
        message.linear.x = -0.05 * qlinear;
        message.angular.z = 0.0 * qraw;
    }
    else if (key == 'j')
    {
        message.linear.x = 0.0 * qlinear;
        message.angular.z = 0.05 * qraw;
    }
    else if (key == 'l')
    {
        message.linear.x = 0.0 * qlinear;
        message.angular.z = -0.05 * qraw;
    }
    else
    {
        message.linear.x = 0.0;
        message.angular.z = 0.0;
    }
    message.linear.y = 0.0;
    message.linear.z = 0.0;
    message.angular.x = 0.0;
    message.angular.y = 0.0;
    CMD_VEL_publisher->publish(message);
}

void rclcomm::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    QPolygonF emit_points;
    for (unsigned int i = 0; i < msg->poses.size(); i++)
    {
        double x = msg->poses.at(i).pose.position.x;
        double y = msg->poses.at(i).pose.position.y;
        QPointF point;
        point.setX(x);
        point.setY(y);
        point = transWordPoint2Scene(point);
        emit_points.push_back(point);
    }
    emit emitUpdatePath(emit_points);
}

void rclcomm::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    double angle_min = msg->angle_min;
    double angle_increment = msg->angle_increment;
    try
    {
        geometry_msgs::msg::PointStamped point_map_frame;
        geometry_msgs::msg::PointStamped point_laser_frame;
        QPolygonF emit_points;
        for (unsigned int i = 0; i < msg->ranges.size(); i++)
        {
            double angle = angle_min + i * angle_increment;

            double x = msg->ranges[i] * cos(angle);
            double y = msg->ranges[i] * sin(angle);
            point_laser_frame.point.x = x;
            point_laser_frame.point.y = y;
            point_laser_frame.header.frame_id = msg->header.frame_id;
            m_tf_buffer->transform(point_laser_frame, point_map_frame, "map");
            QPointF laser_scene_point;
            laser_scene_point.setX(point_map_frame.point.x);
            laser_scene_point.setY(point_map_frame.point.y);
            laser_scene_point = transWordPoint2Scene(laser_scene_point);
            emit_points.push_back(laser_scene_point);
        }
        emit emitUpdateLaserPoint(emit_points);
    }
    catch (tf2::TransformException &ex)
    {
        qDebug() << "laser pose transform error:" << ex.what();
    }
}

void rclcomm::globalCostMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    int width = msg->info.width;
    int height = msg->info.height;
    QImage map_image(width, height, QImage::Format_ARGB32);
    for (unsigned int i = 0; i < msg->data.size(); i++)
    {
        int x = i % width;
        int y = (int)i / width;
        QColor color;
        int data = msg->data[i];
        if (data >= 100)
        {
            color.setRgb(0xff, 0x00, 0xff);
            color.setAlpha(50);
        }
        else if (data >= 90 && data < 100)
        {
            color.setRgb(0x66, 0xff, 0xff);
            color.setAlpha(50);
        }
        else if (data >= 70 && data <= 90)
        {
            color.setRgb(0xff, 0x00, 0x33);
            color.setAlpha(50);
        }
        else if (data >= 60 && data <= 70)
        {
            color.setRgb(0xbe, 0x28, 0x1a); // red
            color.setAlpha(50);
        }
        else if (data >= 50 && data < 60)
        {
            color.setRgb(0xBE, 0x1F, 0x58);
            color.setAlpha(50);
        }
        else if (data >= 40 && data < 50)
        {
            color.setRgb(0xBE, 0x25, 0x76);
            color.setAlpha(50);
        }
        else if (data >= 30 && data < 40)
        {
            color.setRgb(0xBE, 0x2A, 0x99);
            color.setAlpha(50);
        }
        else if (data >= 20 && data < 30)
        {
            color.setRgb(0xBE, 0x35, 0xB3);
            color.setAlpha(50);
        }
        else if (data >= 10 && data < 20)
        {
            color.setRgb(0xB0, 0x3C, 0xbE);
            color.setAlpha(50);
        }
        else
        {
            color = Qt::transparent;
        }
        map_image.setPixelColor(x, y, color);
    }
    map_image = rotateMapWithY(map_image);
    emit emitUpdateGlobalCostMap(map_image);
}

void rclcomm::localCostMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    int width = msg->info.width;
    int height = msg->info.height;
    double origin_x = msg->info.origin.position.x;
    double origin_y = msg->info.origin.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(msg->info.origin.orientation, q);
    tf2::Matrix3x3 mat(q);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    double origin_theta = yaw;
    QImage map_image(width, height, QImage::Format_ARGB32);
    for (unsigned int i = 0; i < msg->data.size(); i++)
    {
        int x = i % width;
        int y = (int)i / width;
        QColor color;
        int data = msg->data[i];
        if (data >= 100)
        {
            color.setRgb(0xff, 00, 0xff);
        }
        else if (data >= 90 && data < 100)
        {
            color.setRgb(0x66, 0xff, 0xff);
        }
        else if (data >= 70 && data <= 90)
        {
            color.setRgb(0xff, 0x00, 0x33);
        }
        else if (data >= 60 && data <= 70)
        {
            color.setRgb(0xbe, 0x28, 0x1a); // red
        }
        else if (data >= 50 && data < 60)
        {
            color.setRgb(0xBE, 0x1F, 0x58);
        }
        else if (data >= 40 && data < 50)
        {
            color.setRgb(0xBE, 0x25, 0x76);
        }
        else if (data >= 30 && data < 40)
        {
            color.setRgb(0xBE, 0x2A, 0x99);
        }
        else if (data >= 20 && data < 30)
        {
            color.setRgb(0xBE, 0x35, 0xB3);
        }
        else if (data >= 10 && data < 20)
        {
            color.setRgb(0xB0, 0x3C, 0xbE);
        }
        else
        {
            color = Qt::transparent;
        }
        map_image.setPixelColor(x, y, color);
    }
    map_image = rotateMapWithY(map_image);
    try
    {
        geometry_msgs::msg::PoseStamped pose_map_frame;
        geometry_msgs::msg::PoseStamped pose_curr_frame;
        pose_curr_frame.pose.position.x = origin_x;
        pose_curr_frame.pose.position.y = origin_y;
        q.setRPY(0, 0, origin_theta);
        pose_curr_frame.pose.orientation = tf2::toMsg(q);
        pose_curr_frame.header.frame_id = msg->header.frame_id;
        m_tf_buffer->transform(pose_curr_frame, pose_map_frame, "map");
        tf2::fromMsg(pose_map_frame.pose.orientation, q);
        tf2::Matrix3x3 mat(q);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        QPointF scene_origin =
            transWordPoint2Scene(QPointF(pose_map_frame.pose.position.x, pose_map_frame.pose.position.y));
        RobotPose localCostmapPose;
        localCostmapPose.x = scene_origin.x();
        localCostmapPose.y = scene_origin.y();
        localCostmapPose.theta = yaw;
        emit emitUpdateLocalCostMap(map_image, localCostmapPose);
    }
    catch (tf2::TransformException &ex)
    {
        qDebug() << "local cost map pose transform error:" << ex.what();
    }
}

void rclcomm::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    double origin_x = msg->info.origin.position.x;
    double origin_y = msg->info.origin.position.y;
    int width = msg->info.width;
    int height = msg->info.height;
    m_resolution = msg->info.resolution;
    QImage map_image(width, height, QImage::Format_RGB32);
    for (unsigned int i = 0; i < msg->data.size(); i++)
    {
        int x = i % width;
        int y = int(i / width);
        QColor color;
        if (msg->data[i] == 100)
        {
            color = Qt::black; // 黑色
        }
        else if (msg->data[i] == 0)
        {
            color = Qt::white; // 白色
        }
        else
        {
            color = Qt::gray;
        }
        map_image.setPixel(x, y, qRgb(color.red(), color.green(), color.blue()));
    }
    QImage rotate_map = rotateMapWithY(map_image);
    emit emitUpdateMap(rotate_map);
    double trans_origin_x = origin_x;
    double trans_origin_y = origin_y + height * m_resolution;
    m_wordOrigin.setX(fabs(trans_origin_x / m_resolution));
    m_wordOrigin.setY(fabs(trans_origin_y / m_resolution));
}

void rclcomm::pub2DPose(QPointF start_pose, QPointF end_pose)
{
    start_pose = transScenePoint2Word(start_pose);
    end_pose = transScenePoint2Word(end_pose);
    double angle = atan2(end_pose.y() - start_pose.y(), end_pose.x() - start_pose.x());
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = node->get_clock()->now();
    pose.pose.pose.position.x = start_pose.x();
    pose.pose.pose.position.y = start_pose.y();
    tf2::Quaternion q;
    q.setRPY(0, 0, angle);
    pose.pose.pose.orientation = tf2::toMsg(q);
    _initPosePublisher->publish(pose);
}

void rclcomm::pub2DGoal(QPointF start_pose, QPointF end_pose)
{
    start_pose = transScenePoint2Word(start_pose);
    end_pose = transScenePoint2Word(end_pose);
    double angle = atan2(end_pose.y() - start_pose.y(), end_pose.x() - start_pose.x());
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = node->get_clock()->now();
    pose.pose.position.x = start_pose.x();
    pose.pose.position.y = start_pose.y();
    tf2::Quaternion q;
    q.setRPY(0, 0, angle);
    pose.pose.orientation = tf2::toMsg(q);
    _navPosePublisher->publish(pose);
}
