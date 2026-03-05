from launch import LaunchDescription
from launch_ros.actions import Node

pub_topic = "/pub_data"

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_talker_ros2',
            node_executable='pub_sub_node',
            node_name='pub_sub_node',
            parameters=[{'pub_topic': pub_topic}],
            output='screen',
        )
    ])