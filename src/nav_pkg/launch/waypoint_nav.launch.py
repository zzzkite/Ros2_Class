import os

# 引入ROS2的Launch框架核心类，用于创建启动描述
from launch import LaunchDescription 

# 引入Node类，用于启动单个ROS2节点
from launch_ros.actions import Node 

# 获取某个已安装包的share目录路径
from ament_index_python.packages import get_package_share_directory

# 用于引入其他 launch 文件
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 获取地图文件路径（用于导航地图加载）
    map_file = os.path.join(
        get_package_share_directory('wpr_simulation2'),  # 包名
        'maps',                                           # maps 文件夹
        'map.yaml'                                        # 地图 yaml 文件
    )

    # 获取导航配置文件路径
    nav_param_file = os.path.join(
        get_package_share_directory('wpr_simulation2'),
        'config',
        'nav2_params.yaml'
    )

    # 获取nav2_bringup包中的launch文件夹路径
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 
        'launch'
    )

    # 启动Nav2导航系统，调用 bringup_launch.py 文件
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav2_launch_dir, '/bringup_launch.py'   # 完整路径拼接
        ]),
        launch_arguments={
            'map': map_file,                        # 地图路径
            'use_sim_time': 'True',                 # 启用仿真时间（Gazebo/仿真环境必开）
            'params_file': nav_param_file           # 配置参数路径
        }.items(),  # 传入参数作为launch_arguments
    )

    # 获取RViz的显示配置文件路径
    rviz_file = os.path.join(
        get_package_share_directory('wp_map_tools'),  # 包名
        'rviz',
        'navi.rviz'                                    # RViz 配置文件
    )

    # 启动RViz可视化界面，加载指定配置
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file]
    )

    # 启动自定义节点：wp_edit_node（一般用于编辑导航点）
    wp_edit_cmd = Node(
        package='wp_map_tools',
        executable='wp_edit_node',
        name='wp_edit_node'
    )

    # 启动自定义节点：wp_navi_server（通常用于从编辑好的路径中执行自动导航）
    wp_navi_server_cmd = Node(
        package='wp_map_tools',
        executable='wp_navi_server',
        name='wp_navi_server'
    )

    # 创建LaunchDescription对象，并添加所有启动动作
    ld = LaunchDescription()
    ld.add_action(navigation_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(wp_edit_cmd)
    ld.add_action(wp_navi_server_cmd)

    return ld  # 返回完整的启动配置
