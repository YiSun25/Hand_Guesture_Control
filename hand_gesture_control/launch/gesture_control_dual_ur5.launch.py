from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    gesture_pkg = get_package_share_directory('hand_gesture_control')  # 
    moveit_pkg = get_package_share_directory('dual_ur5_moveit_config')  # 
    # launch MoveIt RViz demo
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, 'launch', 'demo.launch.py')
        )
    )

    # launch hand gesture node
    hand_gesture_node = Node(
        package='hand_gesture_control',  # package 
        executable='hand_gesture_publisher',  # file name
        name='gesture_publisher',
        output='screen'
    )

    # launch control node
    gesture_control_node = Node(
        package='hand_gesture_control',  # package
        executable='hand_gesture_subscriber', # file name
        name='gesture_subscriber',
        output='screen'
    )

    return LaunchDescription([
        moveit_demo,
        hand_gesture_node,
        gesture_control_node
    ])
