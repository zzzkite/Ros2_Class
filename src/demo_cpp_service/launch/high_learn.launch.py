import launch
import launch.launch_description_sources
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory  # 用于获取 ROS 2 软件包的共享路径

def generate_launch_description():
    """
    生成 ROS 2 Launch Description（启动描述），用于同时启动多个节点或其他 launch 文件。
    """

    # 1️⃣ 声明一个 Launch 参数
    # 这个参数用于控制是否启动 `rqt`，默认值为 'False'
    action_declare_startup_rqt = launch.actions.DeclareLaunchArgument('rqt', default_value='False')

    # 获取 'rqt' 参数的值（如果用户在命令行指定了该参数，则使用用户提供的值，否则使用默认值 'False'）
    startup_rqt_str = launch.substitutions.LaunchConfiguration('rqt', default='False')

    # 2️⃣ 获取需要启动的其他 launch 文件的路径
    # 这里我们获取 `turtlesim` 包中 `multisim.launch.py` 的路径
    multisim_launch_path = os.path.join(get_package_share_directory('turtlesim'), 'launch', 'multisim.launch.py')

    # 3️⃣ 创建 IncludeLaunchDescription 动作
    # 该动作会启动 `multisim.launch.py` 这个 launch 文件
    action_include_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(multisim_launch_path)
    )

    # 4️⃣ 记录日志
    # 这会在终端打印 `multisim.launch.py` 的路径，方便调试
    action_log_info = launch.actions.LogInfo(msg=f"Launching: {multisim_launch_path}")

    # 5️⃣ 启动 `rqt` 工具（如果参数 'rqt' 设为 True，则执行 `rqt` 命令）
    action_rqt = launch.actions.ExecuteProcess(
        condition=launch.conditions.IfCondition(startup_rqt_str),  # 只有当 'rqt' 参数为 'True' 时才会执行
        cmd=['rqt']  # 运行 `rqt` 命令
    )

    # 6️⃣ 组织 Launch 动作
    action_group = launch.actions.GroupAction([
        # 6.1 启动 `multisim.launch.py`（延迟 4 秒后启动）
        launch.actions.TimerAction(period=4.0, actions=[action_include_launch]),

        # 6.2 启动 `rqt`（延迟 2 秒后启动）
        launch.actions.TimerAction(period=2.0, actions=[action_rqt])
    ])

    # 7️⃣ 返回 LaunchDescription，包含所有需要执行的动作
    return launch.LaunchDescription([
        action_declare_startup_rqt,  # 声明 'rqt' 这个参数
        action_log_info,  # 记录日志
        action_group  # 组织所有的 launch 动作
    ])
