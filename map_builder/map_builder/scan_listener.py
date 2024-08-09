import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import numpy as np
import math
from std_msgs.msg import Header

def quaternion_to_rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
    ])

class LaserScanToPointCloud(Node):

    def __init__(self):
        super().__init__('scan_subscriber')

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile
        )
        self.publisher = self.create_publisher(PointCloud2, '/cloud', qos_profile)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
            z = 0.0
            points.append((x, y, z))

        try:
            transform = self.tf_buffer.lookup_transform('odom', 'base_scan', rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform "base_scan" to "odom": {ex}')
            return

        transformed_points = self.transform_points(points, transform)
        self.publish_point_cloud(transformed_points, msg.header.stamp, 'odom')

    def transform_points(self, points, transform):
        transformed_points = []

        # Extract translation and rotation
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # Convert quaternion to a rotation matrix
        rotation_matrix = quaternion_to_rotation_matrix(rotation)

        for point in points:
            x = point[0]
            y = point[1]
            z = point[2]

            # Apply the rotation and translation
            point_transformed = np.dot(rotation_matrix, np.array([x, y, z]))
            x_transformed = point_transformed[0] + translation.x
            y_transformed = point_transformed[1] + translation.y
            z_transformed = point_transformed[2] + translation.z
            transformed_points.append((x_transformed, y_transformed, z_transformed))
        
        return transformed_points

    def publish_point_cloud(self, points, stamp, frame_id):
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        point_cloud_msg = point_cloud2.create_cloud(header, fields, points)
        self.publisher.publish(point_cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()