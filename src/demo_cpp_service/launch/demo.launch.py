import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory #用来获取路径

def generate_launch_description():
    #1.声明一个launch参数
    action_declare_arg_background_g = launch.actions.DeclareLaunchArgument('launch_arg_bg', default_value='250')
    #2.把launch参数手动传给某个节点
    action_node_turtlesim_node = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        parameters=[{'background_g': launch.substitutions.LaunchConfiguration('launch_arg_bg', default='250')}],
        output='screen',
    )
    action_node_turtle_control_service = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='turtle_control_service',
        output='screen',
    )
    action_node_turtle_control_client = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='turtle_control_client',
        output='screen',
    )

    return launch.LaunchDescription([
        action_declare_arg_background_g,
        action_node_turtlesim_node,
        action_node_turtle_control_service,
        action_node_turtle_control_client,
    ])
