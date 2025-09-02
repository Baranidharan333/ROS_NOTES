# talker_listener_launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('py_pubsub')
    #talker_params = os.path.join(pkg_share, 'config', 'talker_params.yaml')

    talker_node = Node(
        package='py_pubsub',
        executable='talker',
        name='talker_node',
        output='screen',
        #parameters=[talker_params]  # load YAML params for talker
    )

    listener_node = Node(
        package='py_pubsub',
        executable='listener',
        name='listener_node',
        output='screen',
    )

    return LaunchDescription([talker_node, listener_node])