import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gscam',
            executable='gscam_node',
            name='gscam',
            output='screen',
            parameters=[{
                'gscam_config': 'videotestsrc pattern=ball ! video/x-raw,width=640,height=480,framrate=10/1 ! videoconvert',
                'use_gst_timestamps': True,
            }]
        )
    ])