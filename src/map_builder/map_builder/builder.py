import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
import struct
import numpy as np

class LaserScanToPointCloudNode(Node):

    def __init__(self):
        super().__init__('laser_scan_to_point_cloud_node')

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_scan_callback,
            qos_profile)
        self.publisher = self.create_publisher(PointCloud2, 'cloud', qos_profile)

    def laser_scan_callback(self, msg):
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                z = 0.0
                points.append([x, y, z])
            angle += msg.angle_increment

        # Usar el encabezado del mensaje LaserScan
        header = msg.header

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        point_cloud_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=12,  # 3 floats (x, y, z) * 4 bytes/float
            row_step=len(points) * 12,
            data=np.array(points, dtype=np.float32).tobytes()
        )

        self.publisher.publish(point_cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()