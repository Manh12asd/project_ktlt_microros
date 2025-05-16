from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    droidcam_publisher_node = Node(
        package='esp32bot_detect',
        executable='droidcam_publisher',
        name='droidcam_publisher_node',
    )
    

    human_detection_node = Node(
        package='esp32bot_detect',
        executable='human_detect',
    )

    rqt = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='rqt_gui',
        output='screen',
    )

    return LaunchDescription([
        droidcam_publisher_node,
        human_detection_node,
        rqt
    ])