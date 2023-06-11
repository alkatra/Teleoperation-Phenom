import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations

class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_frame_publisher')
        broadcaster = StaticTransformBroadcaster(self)
        static_transformStamped = TransformStamped()

        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = 'base_link'
        static_transformStamped.child_frame_id = 'ultrasonic_left'
        static_transformStamped.transform.translation.x = 0.1
        static_transformStamped.transform.translation.y = 0.0
        static_transformStamped.transform.translation.z = 3.0
        quat = tf_transformations.quaternion_from_euler(0.26,0,0.52)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        broadcaster.sendTransform(static_transformStamped)

def main():
    rclpy.init()
    node = StaticFramePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
