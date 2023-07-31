#nwu_to_ned_transforms

## Before running file, ensure the following are installed:
# 'pip install tf2_ros'
# 'pip install geometry_msgs'

## import necessary dependencies
import rclpy # ros client library for python language
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3 #ros message types used for coordnate transformations

def convert_nwu_to_ned(nwu_coordinateS):
    rclpy.init(args=None) # Initialize the rclpy library

    # Create a node for the NWU to NED converter
    # Node acts as container for the coordinate frame conversion functionality
    node = rclpy.create_node('nwu_to_ned_converter')

    ## Create a TF2 Buffer and Listener
    # tf2 buffer <-- initializes container that stores a collection of transformation information
    # tf2 listener <-- componet that subscribes to and recieves messages publishes on a specific topic
    tf_buffer = tf2_ros.Buffer(node) #object created, serves as container  for maintaining the transformation history
    tf_listener = tf2_ros.TransformListener(tf_buffer) #object created, listens for transformation messages on '/tf' topic and populates buffer with transformation data

    try:
        ## Wait for the transformation to be available (waits for the transformation between the "map" and "world" frames to be available)
        # wait_for_transform function <-- blocks execution of transform until available or timeout of 5 seconds
        tf_buffer.wait_for_transform("map", "world", rclpy.time.Time(seconds=0.0), rclpy.duration.Duration(seconds=5.0)) #specifies source frame (map), target frame (world) and obtains transform at 0 seconds.

        # Convert the NWU coordinates to NED using the TF2 library
        ned_transform = tf_buffer.lookup_transform("map", "world", rclpy.time.Time(seconds=0.0), rclpy.duration.Duration(seconds=1.0)) #uses lookup_transform to retrieve transformation data from buffer
        ned_coordinates = tf2_ros.transform_vectors(ned_transform, nwu_coordinates) #applies transformation to input (nwu coordinates) and using ned_transform gives NED coordinates
        return ned_coordinates.vector #returns NED coordinates as Vector3 message
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        node.get_logger().error("Error occurred during coordinate conversion: %s", str(e))
        return None
    finally:
        # Clean up and shutdown the rclpy node
        node.destroy_node()
        rclpy.shutdown()

# Initializes sample NWU coordinates and calls the convert nwu to ned function to convert them to NED coordinates
if __name__ == '__main__':
    # Sample NWU coordinates (you can replace these with your actual coordinates)
    nwu_coordinates = Vector3(x=10.0, y=5.0, z=2.0)

    # Convert NWU to NED coordinates
    ned_coordinates = convert_nwu_to_ned(nwu_coordinates)

    if ned_coordinates:
        print("NED Coordinates:")
        print("North:", ned_coordinates.x)
        print("East:", ned_coordinates.y)
        print("Down:", ned_coordinates.z)
    else:
        print("Failed to convert coordinates.")
