import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from transforms3d.euler import quat2euler


class ImuSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            'imu_topic',  # Replace with the actual topic name
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Extract quaternion from the Imu message
        quaternion = (
            msg.orientation.w,  # Note that transforms3d expects the quaternion in (w, x, y, z) order
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        )

        # Convert quaternion to Euler angles
        roll, pitch, yaw = quat2euler(quaternion)

        # Print the Euler angles
        self.get_logger().info(f'Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}')

def main(args=None):
    rclpy.init(args=args)

    imu_subscriber = ImuSubscriber()

    rclpy.spin(imu_subscriber)

    # Destroy the node explicitly
    imu_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
