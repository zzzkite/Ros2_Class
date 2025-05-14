import os
from launch import LaunchDescription                         # 导入LaunchDescription，用于描述整个启动过程
from launch_ros.actions import Node                          # 导入Node类，用于定义需要启动的ROS 2节点
from ament_index_python.packages import get_package_share_directory  # 用于获取某个包的共享目录（通常用于查找配置或rviz文件）

def generate_launch_description():
    # ------------------------------
    # 配置slam_toolbox参数
    # ------------------------------
    slam_params = {
        "use_sim_time": True,         # 使用仿真时间（通常仿真环境中需要开启）
        "base_frame": "base_footprint",  # 機器人的底盘坐标系
        "odom_frame": "odom",         # 里程计坐标系
        "map_frame": "map" ,           # 地图坐标系（SLAM会构建这个）
        "map_update_interval":1.0
    }

    # ------------------------------
    # 创建slam_toolbox节点
    # 使用sync_slam_toolbox_node节点实现同步SLAM建图
    # ------------------------------
    slam_cmd = Node(
        package="slam_toolbox",               # 指定节点所在包名
        executable="sync_slam_toolbox_node",  # 可执行程序名称
        parameters=[slam_params]              # 启动时加载的参数字典
    )

    # ------------------------------
    # 加载rviz配置文件路径（来自wpr_simulation2包中的rviz目录）
    # ------------------------------
    rviz_file = os.path.join(
        get_package_share_directory('wpr_simulation2'),  # 获取wpr_simulation2包的share目录
        'rviz',                                           # 子目录
        'slam.rviz'                                       # rviz配置文件名
    )

    # ------------------------------
    # 创建rviz2节点，加载自定义的slam.rviz配置
    # ------------------------------
    rviz_cmd = Node(
        package='rviz2',                # 使用rviz2包
        executable='rviz2',             # rviz2主程序
        name='rviz2',                   # 设置节点名称
        arguments=['-d', rviz_file]     # 加载指定的rviz配置文件
    )

    # ------------------------------
    # 创建LaunchDescription并添加节点启动动作
    # ------------------------------
    ld = LaunchDescription()
    ld.add_action(slam_cmd)   # 添加SLAM节点
    ld.add_action(rviz_cmd)   # 添加RViz节点

    return ld                 # 返回Launch描述符，用于ros2 launch执行
