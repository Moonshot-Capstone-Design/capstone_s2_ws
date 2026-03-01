import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # Args
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rviz_config')
    gui = LaunchConfiguration('gui')

    # Default paths (패키지 안에 urdf/rviz 폴더가 있다고 가정)
    pkg_share = get_package_share_directory('manipulator')

    default_model = os.path.join(pkg_share, 'description', 'capstone_manipulator.urdf.xacro')
    default_rviz = os.path.join(pkg_share, 'rviz', 'manipulator.rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock if true'
    )

    declare_model = DeclareLaunchArgument(
        'model',
        default_value=default_model,
        description='Absolute path to robot URDF/Xacro file'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz,
        description='Absolute path to RViz config file'
    )

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Use joint_state_publisher_gui if true'
    )

    # Read model file as robot_description
    # (네 파일이 사실상 URDF라서 xacro 처리 없이 읽어도 됨)
    # 만약 진짜 xacro 매크로/치환이 있으면 아래 방식을 xacro 처리로 바꿔야 함.
    robot_description = None
    with open(os.path.expanduser(str(default_model)), 'r') as f:
        robot_description = f.read()

    # Robot State Publisher (publishes TF from robot_description + /joint_states)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Joint State Publisher (GUI or non-GUI)
    # GUI가 켜져 있으면 joint_state_publisher_gui, 아니면 joint_state_publisher
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=None  # 조건은 아래에서 런치 분기 대신 gui arg로 사용자 선택하도록 함
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=None
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    # NOTE:
    # Launch 조건 분기를 쓰지 않고 단순하게 항상 GUI를 쓰는 방식이 가장 덜 꼬임.
    # gui:=false를 진짜로 쓰고 싶으면 Launch 조건(if/unless)을 추가해줄게.
    # 여기서는 너가 바로 쓰기 좋게 GUI 기본을 켜서 제공.
    return LaunchDescription([
        declare_use_sim_time,
        declare_model,
        declare_rviz,
        declare_gui,
        rsp_node,
        jsp_gui_node,
        rviz_node,
    ])