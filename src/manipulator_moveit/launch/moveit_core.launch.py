import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    # MoveIt launch directory
    launch_dir = os.path.join(
        get_package_share_directory('manipulator_moveit'),
        'launch'
    )

    # RViz config file path
    rviz_config_file = os.path.join(
        get_package_share_directory('manipulator_moveit'),
        'rviz',
        'moveit.rviz'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    ld.add_action(rviz_node)

    # move_group launch include
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'move_group.launch.py')
        )
    )

    ld.add_action(move_group_launch)

    return ld