import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from leo_msgs.msg import WheelOdom
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

class PointCloudMap(Node):

    def __init__(self):
        super().__init__('point_cloud_map')

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(PointCloud2, '/cloud', self.cloud_callback, qos_profile)
        self.subscription = self.create_subscription(WheelOdom, '/firmware/wheel_odom', self.odom_callback, qos_profile)

        self.publisher = self.create_publisher(PointCloud2, '/map', qos_profile)
        self.accumulated_points = []
        self.points = []

        self.last_x = 0.0
        self.last_y = 0.0
        self.last_yaw = 0.0
        self.x = None
        self.y = None
        self.yaw = None

        self.stamp = None
        self.frame_id = None

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.x is not None and self.y is not None and self.yaw is not None:
            print(self.x - self.last_x)
            if abs(self.x - self.last_x) > 0.005 or abs(self.y - self.last_y) > 0.005 or abs(self.yaw - self.last_yaw) > 0.01:
                self.accumulated_points.extend(self.points)
                self.publish_accumulated_points(self.stamp, self.frame_id)
                self.last_x = self.x
                self.last_y = self.y
                self.last_yaw = self.yaw

    def odom_callback(self, msg):
        self.x = msg.pose_x
        self.y = msg.pose_y
        self.yaw = msg.pose_yaw

    def cloud_callback(self, msg):
        # Convert PointCloud2 message to a list of points
        self.points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        self.stamp = msg.header.stamp
        self.frame_id = msg.header.frame_id

    def publish_accumulated_points(self, stamp, frame_id):
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id

        fields = [
            point_cloud2.PointField(name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1),
        ]

        point_cloud_msg = point_cloud2.create_cloud(header, fields, self.accumulated_points)
        self.publisher.publish(point_cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
