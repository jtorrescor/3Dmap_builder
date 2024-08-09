import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FrameListener(Node):

    def __init__(self):
        super().__init__('frame_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Llama a la función on_timer cada segundo
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'base_scan',
                'odom',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform "base_scan" to "odom": {ex}')
            return
        print(f'translation: {t.transform.translation}, rotation: {t.transform.rotation}')

def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()