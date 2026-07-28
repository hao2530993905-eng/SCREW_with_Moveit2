from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    start_rviz = LaunchConfiguration("start_rviz")
    attach_tool = LaunchConfiguration("attach_tool")
    load_saved_scene = LaunchConfiguration("load_saved_scene")

    joint_limits = PathJoinSubstitution([
        FindPackageShare("ur_description"),
        "config",
        ur_type,
        "joint_limits.yaml",
    ])
    physical_params = PathJoinSubstitution([
        FindPackageShare("ur_description"),
        "config",
        ur_type,
        "physical_parameters.yaml",
    ])
    visual_params = PathJoinSubstitution([
        FindPackageShare("ur_description"),
        "config",
        ur_type,
        "visual_parameters.yaml",
    ])
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ur_description"),
            "urdf",
            "ur.urdf.xacro",
        ]),
        " robot_ip:=unused",
        " joint_limit_params:=",
        joint_limits,
        " kinematics_params:=",
        kinematics_params_file,
        " physical_params:=",
        physical_params,
        " visual_params:=",
        visual_params,
        " safety_limits:=true",
        " safety_pos_margin:=0.15",
        " safety_k_position:=20",
        " name:=ur",
        " ur_type:=",
        ur_type,
        " script_filename:=ros_control.urscript",
        " input_recipe_filename:=rtde_input_recipe.txt",
        " output_recipe_filename:=rtde_output_recipe.txt",
        " prefix:=\"\"",
    ])
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }

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
                    "launch_servo": "false",
                }.items(),
            ),
        ],
    )

    scene_actions = TimerAction(
        period=3.0,
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
            "kinematics_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("ur_description"),
                "config",
                ur_type,
                "default_kinematics.yaml",
            ]),
        ),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("attach_tool", default_value="true"),
        DeclareLaunchArgument("load_saved_scene", default_value="true"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        moveit_group,
        scene_actions,
    ])
