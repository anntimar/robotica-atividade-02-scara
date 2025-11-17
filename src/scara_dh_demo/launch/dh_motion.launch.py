from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(package='scara_dh_demo', executable='dh_motion_demo', name='dh_motion_demo', output='screen')
    ])
