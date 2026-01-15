import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction, LogInfo
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
# 引入获取包路径的库
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # =========================================================================
    # 1. 配置 
    # =========================================================================
    
    DESCRIPTION_PACKAGE = "openarm_moveit_config"
    # 确保 URDF 存在于 install/share/openarm_moveit_config/config/ 下
    DESCRIPTION_FILE = "mujoco_openarm.urdf.xacro" 
    
    pkg_share_config = get_package_share_directory(DESCRIPTION_PACKAGE)
    xacro_file_path = os.path.join(pkg_share_config, "config", DESCRIPTION_FILE)

    # --- 关键修改：动态获取 scene.xml 路径 ---
    # 假设你的模型包叫 openarm_mujoco (或 openarm_mujuco)
    # 并且在 CMakeLists.txt 中安装了 v1 文件夹到 share 目录
    try:
        # 优先尝试标准拼写
        pkg_share_mujoco = get_package_share_directory('openarm_mujoco')
        mujoco_model_path = os.path.join(pkg_share_mujoco, 'v1', 'scene.xml')
    except:
        try:
            # 尝试用户可能的拼写 openarm_mujuco
            pkg_share_mujoco = get_package_share_directory('openarm_mujuco')
            mujoco_model_path = os.path.join(pkg_share_mujoco, 'v1', 'scene.xml')
        except:
             # 如果未安装，可能需要硬编码回退 (仅用于调试)
             print("警告: 无法找到 openarm_mujoco 包，请确保已 source install/setup.bash")
             # 这里可以保留一个绝对路径作为最后的 fallback，或者直接报错
             mujoco_model_path = "/请确认_openarm_mujoco_包已正确安装/v1/scene.xml"

    print(f"Loading MuJoCo Model from: {mujoco_model_path}")

    MOVEITCONTROLLERS_FILE = "moveit_controllers.yaml"
    moveit_controller_config_file_path = os.path.join(pkg_share_config, "config", MOVEITCONTROLLERS_FILE)
    MUJOCO_CONTROLLERS_FILE = "ros2_controllers.yaml"
    mujoco_controller_config_file_path = os.path.join(pkg_share_config, "config", MUJOCO_CONTROLLERS_FILE)

    # =========================================================================
    # 生成 MoveIt 配置
    # =========================================================================
    moveit_config = MoveItConfigsBuilder("openarm", package_name=DESCRIPTION_PACKAGE) \
        .robot_description(file_path=xacro_file_path) \
        .trajectory_execution(file_path=moveit_controller_config_file_path) \
        .to_moveit_configs()

    use_sim_time = {"use_sim_time": True}
    TIMEOUT_SEC = "60"

    # =========================================================================
    # 2. 基础节点
    # =========================================================================
    
    # 静态 TF (World -> Base)
    node_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link', '--ros-args', '-p', 'use_sim_time:=true'],
    )

    # MuJoCo
    node_mujoco = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            mujoco_controller_config_file_path,
            use_sim_time,
            {'mujoco_model_path': mujoco_model_path}
        ]
    )

    # Robot State Publisher
    node_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description, use_sim_time]
    )

    # Move Group
    node_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            use_sim_time,
            {
                "publish_robot_description_semantic": True,
                "moveit_manage_controllers": True,
                "monitor_dynamics": False,
                "trajectory_execution.allowed_execution_duration_scaling": 2.0,
                "trajectory_execution.allowed_goal_duration_margin": 0.5,
                "trajectory_execution.allowed_start_tolerance": 0.5,
            },
        ],
    )

    # RViz
    rviz_config = PathJoinSubstitution([FindPackageShare(DESCRIPTION_PACKAGE), "config", "moveit.rviz"])
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            use_sim_time,
        ],
    )

    # =========================================================================
    # 3. 控制器 Spawner
    # =========================================================================
    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_jsb",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "60"],
        output="screen",
    )

    spawner_left_arm = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_left_arm",
        arguments=["left_arm_controller", "--controller-manager", "/controller_manager", "--controller-manager-timeout", TIMEOUT_SEC],
        output="screen",
    )

    spawner_right_arm = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_right_arm",
        arguments=["right_arm_controller", "--controller-manager", "/controller_manager", "--controller-manager-timeout", TIMEOUT_SEC],
        output="screen",
    )

    spawner_left_hand = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_left_hand",
        arguments=["left_hand_controller", "--controller-manager", "/controller_manager", "--controller-manager-timeout", TIMEOUT_SEC],
        output="screen",
    )

    spawner_right_hand = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_right_hand",
        arguments=["right_hand_controller", "--controller-manager", "/controller_manager", "--controller-manager-timeout", TIMEOUT_SEC],
        output="screen",
    )

    # =========================================================================
    # 4. 启动顺序管理
    # =========================================================================
    nodes = [node_static_tf, node_mujoco, node_rsp, node_move_group, node_rviz]

    # MuJoCo -> JSB
    nodes.append(RegisterEventHandler(
        event_handler=OnProcessStart(target_action=node_mujoco, on_start=[TimerAction(period=10.0, actions=[LogInfo(msg="启动 JSB..."), spawner_jsb])])
    ))

    # JSB -> Left Arm
    nodes.append(RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawner_jsb, on_exit=[LogInfo(msg="启动 Left Arm..."), spawner_left_arm])
    ))

    # Left Arm -> Right Arm
    nodes.append(RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawner_left_arm, on_exit=[LogInfo(msg="启动 Right Arm..."), spawner_right_arm])
    ))

    # Right Arm -> Left Hand
    nodes.append(RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawner_right_arm, on_exit=[LogInfo(msg="启动 Left Hand..."), spawner_left_hand])
    ))

    # Left Hand -> Right Hand
    nodes.append(RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawner_left_hand, on_exit=[LogInfo(msg="启动 Right Hand..."), spawner_right_hand])
    ))

    return LaunchDescription(nodes)
