from os import path
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    iplow_description = get_package_share_path("iplow_description")
    iplow_bringup = get_package_share_path("iplow_bringup")
    exwayz = get_package_share_path("exwayz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_robot_state = LaunchConfiguration("publish_robot_state")
    odom_to_tf2_config = LaunchConfiguration("odom_to_tf2_config")
    reloc_config = LaunchConfiguration("reloc_config")
    pose_generator_config = LaunchConfiguration("pose_generator_config")

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(iplow_description/ "launch" / "robot_state_publisher.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(publish_robot_state),
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    odom_to_tf2 = Node(
        package="iplow_bringup",
        executable="odom_to_tf2",
        name="odom_to_tf",
        output="screen",
        parameters=[odom_to_tf2_config],
        remappings=[],
    )

    livox_MID360 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(iplow_description/ "launch" / "MID360.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(publish_robot_state),
    )

    pose_generator_node = Node(
        package="exwayz",
        executable="ros2_pose_generator",
        name="ros2_pose_generator",
        parameters=[
            LaunchConfiguration("pose_generator_config"),
        ]
    )

    reloc = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            str(exwayz / "launch" / "reloc.launch")
        ),
        launch_arguments={
            "config": reloc_config,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use sim time if true",
            ),
            DeclareLaunchArgument(
                "publish_robot_state",
                default_value="true",
                description="Run robot state publisher if true",
            ),
            DeclareLaunchArgument(
                "reloc_config",
                default_value=str(iplow_bringup / "config" / "relocalization.yaml"),
                description="Exwayz relocalization config path",
            ),
            DeclareLaunchArgument(
                "odom_to_tf2_configm",
                default_value=str(iplow_bringup / "config" / "odom_to_tf2.yaml"),
                description="Exwayz relocalization config path",
            ),
            DeclareLaunchArgument(
                "pose_generator_config",
                default_value=str(iplow_bringup / "config" / "pose_generator.yaml"),
                description="Exwayz pose generator config path",
            ),
            robot_state_publisher_launch,
            joint_state_publisher_node,
            pose_generator_node,
            odom_to_tf2,
            reloc,
        ]
    )
