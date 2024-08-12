import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LaserScanListener(Node):

    def __init__(self):
        super().__init__('laser_scan_listener')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        points = []
        for i, range_val in enumerate(ranges):
            if range_val < msg.range_min or range_val > msg.range_max:
                continue
            angle = angle_min + i * angle_increment
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)
            points.append((x, y))
        
        self.get_logger().info(f'Extracted {len(points)} points.')
        for point in points:
            self.get_logger().info(f'Point: x={point[0]}, y={point[1]}')

def main(args=None):
    rclpy.init(args=args)
    laser_scan_listener = LaserScanListener()
    rclpy.spin(laser_scan_listener)

    laser_scan_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
