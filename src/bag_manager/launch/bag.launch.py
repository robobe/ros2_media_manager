from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

PKG = "bag_manager"
PARAM_FILE = "bag.yaml"

def generate_launch_description():
    ld = LaunchDescription()
    
    config_file = PathJoinSubstitution([
        get_package_share_directory(PKG),
        'config',
        PARAM_FILE
    ])

    node = Node(
        package='bag_manager',
        executable='my_bag.py',
        output='screen',
        parameters=[config_file]
    )

    ld.add_action(node)

    return ld