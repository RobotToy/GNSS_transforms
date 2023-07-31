## ned_transform_node.py
# listens to GNSS and IMY messages, computes NED coordinate,s and publishes the transformation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class NEDTransformNode(Node):
    def __init__(self):
        super().__init__('ned_transform_node')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.get_logger().info('NED Transform Node initialized.')

        # Subscribe to GNSS and IMU topics
        self.create_subscription(
            # Replace 'gnss_msg_type' with the GNSS message type (e.g., sensor_msgs.msg.NavSatFix)
            gnss_msg_type,
            'gnss_topic',            # Replace with the actual GNSS topic name
            self.gnss_callback,
            10
        )
        self.create_subscription(
            # Replace 'imu_msg_type' with the IMU message type (e.g., sensor_msgs.msg.Imu)
            imu_msg_type,
            'imu_topic',             # Replace with the actual IMU topic name
            self.imu_callback,
            10
        )

        # Initialize variables to store GNSS and IMU data
        self.gnss_data = None
        self.imu_data = None

    def gnss_callback(self, msg):
        # Store the GNSS data for later use
        self.gnss_data = msg

    def imu_callback(self, msg):
        # Store the IMU data for later use
        self.imu_data = msg

        # When both GNSS and IMU data are available, compute NED and publish the transform
        if self.gnss_data is not None:
            ned_transform = self.compute_ned(self.gnss_data, self.imu_data)
            self.publish_transform(ned_transform)

    def compute_ned(self, gnss_data, imu_data):
        # Extract GNSS data
        latitude = gnss_data.latitude
        longitude = gnss_data.longitude
        altitude = gnss_data.altitude

        # Extract IMU orientation (as a quaternion)
        quaternion = imu_data.orientation

        # Assume the origin latitude and longitude (e.g., where the boat started)
        origin_latitude = 0  # TODO: Replace with the latitude of your origin
        origin_longitude = 0  # TODO: Replace with the longitude of your origin

        # Convert latitudes and longitudes from degrees to radians
        latitude_rad = math.radians(latitude)
        longitude_rad = math.radians(longitude)
        origin_latitude_rad = math.radians(origin_latitude)
        origin_longitude_rad = math.radians(origin_longitude)

        # Define the radius of the Earth (approximately)
        earth_radius = 6371000.0  # in meters

        # Compute the NED coordinate (North-East-Down) with respect to the origin
        delta_latitude = latitude_rad - origin_latitude_rad
        delta_longitude = longitude_rad - origin_longitude_rad

        north = earth_radius * delta_latitude
        east = earth_radius * math.cos(origin_latitude_rad) * delta_longitude
        down = altitude  # Assuming the altitude is relative to the origin point

        # Create the transform message
        ned_transform = TransformStamped()
        ned_transform.header.stamp = self.get_clock().now().to_msg()
        ned_transform.header.frame_id = "gnss_frame"  # Replace with your GNSS frame
        ned_transform.child_frame_id = "ned_frame"    # Replace with your NED frame
        ned_transform.transform.translation.x = north
        ned_transform.transform.translation.y = east
        ned_transform.transform.translation.z = -down  # NED is Down-positive, ROS TF is Up-positive

        # Assuming the IMU's orientation (quaternion) directly aligns with the NED frame
        ned_transform.transform.rotation = quaternion

        return ned_transform

    def publish_transform(self, transform_msg):
        # Publish the NED transform
        self.tf_broadcaster.sendTransform(transform_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NEDTransformNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
