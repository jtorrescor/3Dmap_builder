# Publicacion de odometria de Leo Rover con mensaje nav_msgs/Odometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
import rclpy
from rclpy.node import Node
from leo_msgs.msg import WheelOdom

class WheelOdomConverter(Node):
    def __init__(self):
        super().__init__('wheel_odom_converter')

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(
            WheelOdom,
            '/firmware/wheel_odom',
            self.wheel_odom_callback,
            qos_profile
        )
        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)

    def wheel_odom_callback(self, wheel_odom_msg):
        odom_msg = Odometry()
        
        # Asignar la marca de tiempo
        odom_msg.header.stamp = wheel_odom_msg.stamp
        
        # Asignar el frame_id y child_frame_id si es necesario
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        # Asignar la posición y la orientación
        odom_msg.pose.pose.position.x = wheel_odom_msg.pose_x
        odom_msg.pose.pose.position.y = wheel_odom_msg.pose_y
        odom_msg.pose.pose.position.z = 0.0  
        
        # Asignar velocidades lineales y angulares
        odom_msg.twist.twist.linear.x = wheel_odom_msg.velocity_lin
        odom_msg.twist.twist.linear.y = 0.0  
        odom_msg.twist.twist.linear.z = 0.0  
        odom_msg.twist.twist.angular.x = 0.0  
        odom_msg.twist.twist.angular.y = 0.0  
        odom_msg.twist.twist.angular.z = wheel_odom_msg.velocity_ang

        self.publisher_.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    wheel_odom_converter = WheelOdomConverter()
    rclpy.spin(wheel_odom_converter)
    wheel_odom_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
