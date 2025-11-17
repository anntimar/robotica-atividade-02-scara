from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scara_dh_demo',
            executable='scara_traj_demo',
            name='scara_traj_demo',
            output='screen'
        )
    ])
