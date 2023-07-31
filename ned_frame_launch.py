## ned_frame_launch (place inside package named: "ned_frame")
# starts the GNSS and IMU nodes along with the NED frame transformation node

## Assumptions about data
# assumes ZED IMU data publishing to ROS2 topic: "/zed/imu"
# assumes GNSS receiver rpvoding position data in ROS2 topic: "/gnss/position"

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # get the path to the configruation files in the package
    ned_frame_dir = get_package_share_directory('ned_frame')

    return LaunchDescription([
        Node(
            package='your_gnss_package',      # Replace with the GNSS package name
            executable='gnss_node',           # Replace with the GNSS node name
            name='gnss_node',
            output='screen',
        ),
        Node(
            package='your_imu_package',       # Replace with the IMU package name
            executable='imu_node',            # Replace with the IMU node name
            name='imu_node',
            output='screen',
        ),
        Node(
            package='ned_frame',
            executable='ned_transform_node',
            name='ned_transform_node',
            output='screen',
            parameters=[os.path.join(ned_frame_dir, 'params', 'ned_frame_params.yaml')],
        ),
    ])
