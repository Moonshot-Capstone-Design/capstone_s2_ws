from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    marker_topic = LaunchConfiguration("marker_topic")
    cmd_topic = LaunchConfiguration("cmd_topic")
    target_frame = LaunchConfiguration("target_frame")
    ee_link = LaunchConfiguration("ee_link")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "marker_topic",
                default_value="/object_3d_marker",
                description="3D marker topic from perception node",
            ),
            DeclareLaunchArgument(
                "cmd_topic",
                default_value="/manipulator_manager/cmd",
                description="command topic (std_msgs/String)",
            ),
            DeclareLaunchArgument(
                "target_frame",
                default_value="link1",
                description="planning frame used by MoveIt",
            ),
            DeclareLaunchArgument(
                "ee_link",
                default_value="ee_link",
                description="end-effector link name",
            ),
            Node(
                package="manipulator_manager",
                executable="marker_command_manager",
                output="screen",
                parameters=[
                    {
                        "marker_topic": marker_topic,
                        "cmd_topic": cmd_topic,
                        "target_frame": target_frame,
                        "ee_link": ee_link,
                    }
                ],
            ),
        ]
    )

