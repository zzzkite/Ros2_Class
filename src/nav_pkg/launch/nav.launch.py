import os

# 导入 LaunchDescription，用于描述启动配置
from launch import LaunchDescription 

# 导入 Node，用于启动单个 ROS2 节点
from launch_ros.actions import Node 

# 获取某个已安装ROS包的共享目录（如获取配置文件的绝对路径）
from ament_index_python.packages import get_package_share_directory

# 导入 IncludeLaunchDescription，用于嵌套调用其他的 launch 文件
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取地图文件（yaml 格式），用于导航地图加载
    map_file = os.path.join(
        get_package_share_directory('wpr_simulation2'),
        'maps',
        'map.yaml'
    )

    # 获取导航参数配置文件，用于配置导航系统参数（如planner、controller等）
    nav_param_file = os.path.join(
        get_package_share_directory('wpr_simulation2'),
        'config',
        'nav2_params.yaml'
    )

    # 获取 nav2_bringup 包中 launch 文件夹路径，用于调用 bringup_launch.py 启动 Nav2 系统
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 
        'launch'
    )

    # 启动 Nav2 系统（调用 nav2_bringup 的 bringup_launch.py 文件）
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_dir, '/bringup_launch.py']),
        launch_arguments={
            'map': map_file,               # 地图文件路径
            'use_sim_time': 'True',        # 启用仿真时间（Gazebo 或模拟器）
            'params_file': nav_param_file  # 导航参数配置文件
        }.items(),
    )

    # 加载 RViz 可视化界面，加载自定义的导航显示配置
    rviz_file = os.path.join(
        get_package_share_directory('wpr_simulation2'),
        'rviz',
        'navi.rviz'
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file]  # 加载 RViz 配置文件
    )

    # 创建 LaunchDescription 并添加两个动作（Nav2系统和RViz可视化）
    ld = LaunchDescription()
    ld.add_action(navigation_cmd)
    ld.add_action(rviz_cmd)

    return ld
