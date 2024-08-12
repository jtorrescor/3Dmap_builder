import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from sensor_msgs.msg import Imu, LaserScan
from leo_msgs.msg import WheelOdom
from scipy.spatial.transform import Rotation as R
#from tf_transformations import euler_from_quaternion
import numpy as np
import math
import time

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(WheelOdom, '/firmware/wheel_odom', self.odom_callback, qos_profile)
        self.subscription = self.create_subscription(Imu, '/vectornav/imu', self.imu_callback, qos_profile)

        self.target_x = 1.0  # Coordenada X objetivo
        self.target_y = 1.0  # Coordenada Y objetivo
        self.vel_linear_max = 0.5
        self.vel_angular_max = 1.0
        self.k_r = 1
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.time = 0
        self.t = 0

    def timer_callback(self):
        prev_time = self.time
        self.time = time.time()
        dt = self.time - prev_time
        self.t = self.t + dt

        self.target_x = 1 * np.cos(0.1 * self.t)
        self.target_y = 1 * np.sin(0.1 * self.t)

        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        d_error = math.sqrt(error_x**2 + error_y**2)
        yaw_target = math.atan2(error_y, error_x)

        yaw_error = math.atan2(np.sin(yaw_target - self.current_yaw), np.cos(yaw_target - self.current_yaw))

        angular_speed = self.vel_angular_max * np.sin(yaw_target - self.current_yaw)  #self.kp_angular * yaw_error
        #linear_speed = self.kp_linear * d_error
        #angular_speed = max(min(angular_speed, 0.5), -0.5)
        #linear_speed = max(min(linear_speed, 0.5), -0.5)

        if d_error >= self.k_r:
            linear_speed = self.vel_linear_max
        else:
            linear_speed = d_error * (self.vel_linear_max/self.k_r)

        msg = Twist()

        if -math.pi/2 <= yaw_error <= math.pi/2:
            msg.linear.x = linear_speed
        else:
            msg.linear.x = 0.0

        msg.angular.z = angular_speed


        self.publisher_.publish(msg)

    def odom_callback(self, msg):
        self.current_x = msg.pose_x
        self.current_y = msg.pose_y
        self.current_yaw = msg.pose_yaw

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        r = R.from_quat[orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        eulZYX = r.as_euler('zyx')
        self.current_imu_yaw = eulZYX[0]
        print()

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)

    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()