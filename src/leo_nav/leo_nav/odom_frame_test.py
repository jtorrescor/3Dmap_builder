import math
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

class FramePublisher(Node):

    def __init__(self):
        super().__init__('odom_frame_publisher')

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # callback function on each message
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.subscription = self.create_subscription(Imu, '/imu', self.imu_callback, qos_profile)

        self.subscription  # prevent unused variable warning 
        timer_period = 0.0333  # seconds
        self.timer = self.create_timer(timer_period, self.handle_pose)

        # Variables globales
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 0.0

        

    def handle_pose(self):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom1'

        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = self.pose_x
        t.transform.translation.y = self.pose_y
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        t.transform.rotation.x = self.qx
        t.transform.rotation.y = self.qy
        t.transform.rotation.z = self.qz
        t.transform.rotation.w = self.qw

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

    def odom_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        #self.current_yaw = pose_yaw

    def imu_callback(self, msg):
        self.qx = msg.orientation.x
        self.qy = msg.orientation.y
        self.qz = msg.orientation.z
        self.qw = msg.orientation.w

def main(args=None):
    rclpy.init(args=args)
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()