import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_path = get_package_share_directory("brick_wall")

    # ✅ ARG (THIS WAS MISSING BEFORE)
    use_sim = LaunchConfiguration("use_sim")
    enable_joint_state_gui = LaunchConfiguration("enable_joint_state_gui")
    enable_gap_detector = LaunchConfiguration("enable_gap_detector")
    enable_twist_mux = LaunchConfiguration("enable_twist_mux")
    stop_on_gap = LaunchConfiguration("stop_on_gap")
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")
    robot_z = LaunchConfiguration("robot_z")

    # ================= XACRO =================
    wall_xacro = os.path.join(pkg_path, "urdf", "wall.xacro")
    wall_description = xacro.process_file(wall_xacro).toxml()

    robot_xacro = os.path.join(pkg_path, "urdf", "four_wheel_robot.xacro")
    robot_description = ParameterValue(Command(["xacro ", robot_xacro]), value_type=str)

    # ================= GAZEBO =================
    gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
        ],
        condition=IfCondition(use_sim),
        output="screen",
    )

    # ================= STATE PUBLISHERS =================
    wall_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="wall_state_publisher",
        output="screen",
        parameters=[{"robot_description": wall_description, "use_sim_time": use_sim}],
        remappings=[("/robot_description", "/wall_description")],
        condition=IfCondition(use_sim),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim,
            }
        ],
    )

    # ================= SPAWN =================
    spawn_wall = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "brick_wall",
            "-topic",
            "/wall_description",
            "-z",
            "0.0",
        ],
        condition=IfCondition(use_sim),
        output="screen",
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "my_robot",
            "-x",
            robot_x,
            "-y",
            robot_y,
            "-z",
            robot_z,
        ],
        condition=IfCondition(use_sim),
        output="screen",
    )

    # ================= CONTROLLERS =================
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_sim),
        output="screen",
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_sim),
        output="screen",
    )

    # ================= RVIZ MODE =================
    joint_state_gui_non_sim = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_sim,
                    "' == 'false' and '",
                    enable_joint_state_gui,
                    "' == 'true'",
                ]
            )
        ),
        output="screen",
    )

    # In Gazebo mode, publish GUI slider joints on a separate topic to avoid
    # conflicting with ros2_control joint_state_broadcaster on /joint_states.
    joint_state_gui_sim = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        remappings=[("/joint_states", "/joint_states_gui")],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_sim,
                    "' == 'true' and '",
                    enable_joint_state_gui,
                    "' == 'true'",
                ]
            )
        ),
        output="screen",
    )

    joint_state_gui_bridge = Node(
        package="brick_wall",
        executable="joint_state_gui_bridge",
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_sim,
                    "' == 'true' and '",
                    enable_joint_state_gui,
                    "' == 'true'",
                ]
            )
        ),
        output="screen",
    )

    gap_detector = Node(
        package="brick_wall",
        executable="gap_detector",
        parameters=[
            {
                "stop_on_gap": stop_on_gap,
                "cmd_vel_topic": "/cmd_vel_safety",
                "depth_topic": "/depth_camera/depth/image_raw",
                "camera_info_topic": "/depth_camera/depth/camera_info",
                "block_size_m": 0.025,
                "gap_size_m": 0.003,
                "depth_jump_m": 0.004,
                "min_col_gap_ratio": 0.10,
                "min_run_ratio": 0.18,
                "max_gap_depth_m": 0.80,
                "edge_threshold": 10.0,
                "use_rgb_primary": False,
                "roi_row_start": 0.52,
                "roi_row_end": 0.94,
                "roi_col_start": 0.20,
                "roi_col_end": 0.80,
                "gap_confirm_frames": 3,
                "clear_confirm_frames": 3,
                "center_deadband_px": 6.0,
                "center_alpha": 0.22,
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_sim,
                    "' == 'true' and '",
                    enable_gap_detector,
                    "' == 'true'",
                ]
            )
        ),
        output="screen",
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[
            os.path.join(pkg_path, "config", "twist_mux.yaml"),
            {"use_sim_time": use_sim},
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    use_sim,
                    "' == 'true' and '",
                    enable_twist_mux,
                    "' == 'true'",
                ]
            )
        ),
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(pkg_path, "config", "view.rviz")],
        parameters=[{"use_sim_time": use_sim}],
        condition=IfCondition(use_sim),
        output="screen",
    )

    # Bridge simulation world and robot odometry trees for RViz fixed-frame transforms.
    world_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
        condition=IfCondition(use_sim),
        output="screen",
    )

    # ================= FINAL =================
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim", default_value="true"),
            DeclareLaunchArgument("enable_joint_state_gui", default_value="false"),
            DeclareLaunchArgument("enable_gap_detector", default_value="true"),
            DeclareLaunchArgument("enable_twist_mux", default_value="true"),
            DeclareLaunchArgument("stop_on_gap", default_value="false"),
            DeclareLaunchArgument("robot_x", default_value="-35.0"),
            DeclareLaunchArgument("robot_y", default_value="3.0"),
            DeclareLaunchArgument("robot_z", default_value="0.50"),
            gazebo,
            wall_state_publisher,
            robot_state_publisher,
            world_to_odom_tf,
            # RVIZ runs in simulation mode with the configured fixed frame
            joint_state_gui_non_sim,
            TimerAction(
                period=8.0,
                actions=[
                    rviz,
                    twist_mux,
                    joint_state_gui_sim,
                    joint_state_gui_bridge,
                    gap_detector,
                ],
            ),
            # GAZEBO MODE
            TimerAction(period=5.0, actions=[spawn_wall]),
            TimerAction(period=7.0, actions=[spawn_robot]),
            TimerAction(period=9.0, actions=[joint_state_broadcaster]),
            TimerAction(period=10.0, actions=[arm_controller]),
        ]
    )
