from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    start_rviz = LaunchConfiguration("start_rviz")
    launch_servo = LaunchConfiguration("launch_servo")
    attach_tool = LaunchConfiguration("attach_tool")
    load_saved_scene = LaunchConfiguration("load_saved_scene")

    control_group = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("ur_robot_driver"),
                        "launch",
                        "ur_control.launch.py",
                    ])
                ),
                launch_arguments={
                    "ur_type": ur_type,
                    "robot_ip": robot_ip,
                    "use_fake_hardware": use_fake_hardware,
                    "kinematics_params_file": kinematics_params_file,
                    "launch_rviz": "false",
                }.items(),
            ),
        ],
    )

    moveit_group = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("screw_moveit_integration"),
                        "launch",
                        "moveit_with_calibration.launch.py",
                    ])
                ),
                launch_arguments={
                    "ur_type": ur_type,
                    "kinematics_params_file": kinematics_params_file,
                    "launch_rviz": start_rviz,
                    "launch_servo": launch_servo,
                }.items(),
            ),
        ],
    )

    scene_actions = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="ur3_workcell_scene",
                executable="attach_screwdriver",
                output="screen",
                condition=IfCondition(attach_tool),
            ),
            Node(
                package="ur3_workcell_scene",
                executable="restore_saved_obstacles",
                output="screen",
                condition=IfCondition(load_saved_scene),
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3"),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.5",
            description="Real UR controller IP; ignored for fake hardware.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Set true only for simulation/testing.",
        ),
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("ur_description"),
                "config",
                ur_type,
                "default_kinematics.yaml",
            ]),
            description="Factory calibration YAML extracted from the robot.",
        ),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("launch_servo", default_value="true"),
        DeclareLaunchArgument("attach_tool", default_value="true"),
        DeclareLaunchArgument("load_saved_scene", default_value="true"),
        control_group,
        moveit_group,
        scene_actions,
    ])
