import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_cloud2

class ScanToPointCloudNode(Node):

    def __init__(self):
        super().__init__('scan_to_pointcloud_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos_profile = QoSProfile(depth=10)
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile)
        self.cloud_publisher = self.create_publisher(
            PointCloud2,
            'cloud',
            qos_profile)

    async def scan_callback(self, scan_msg):
        try:
            transform = await self.tf_buffer.lookup_transform_async(
                'map', 'base_scan', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warning(f"No se pudo obtener la transformación de base_scan a map: {e}")
            return

        # Crear un PointCloud2 vacío
        cloud_out = PointCloud2()
        cloud_out.header = scan_msg.header  # Copiar el encabezado del LaserScan
        cloud_out.height = 1  # Nube de puntos 2D
        cloud_out.width = len(scan_msg.ranges)  # Número de puntos
        cloud_out.fields = []  # Campos de datos, requeridos
        cloud_out.is_bigendian = False  # El orden de los bytes en los datos
        cloud_out.point_step = 0  # Tamaño en bytes de cada punto
        cloud_out.row_step = 0  # Número total de bytes para la matriz completa
        cloud_out.data = bytes()  # Vector de datos, requerido
        cloud_out.is_dense = False  # No contiene puntos NaN

        # Convertir las lecturas de LaserScan a una nube de puntos en el marco map
        points = []
        for i, range in enumerate(scan_msg.ranges):
            if range != float('inf'):  # Si el rango es válido
                point = PointStamped()
                point.header = scan_msg.header
                point.point.x = range * math.cos(scan_msg.angle_min + i * scan_msg.angle_increment)
                point.point.y = range * math.sin(scan_msg.angle_min + i * scan_msg.angle_increment)
                point.point.z = 0.0  # En 2D, la nube de puntos tiene z = 0
                points.append(point)

        transformed_points = []
        for point in points:
            try:
                transformed_point = do_transform_cloud2(point, transform)
                transformed_points.append(transformed_point)
            except Exception as e:
                self.get_logger().warning(f"No se pudo transformar el punto: {e}")

        # Convertir los puntos transformados a PointCloud2
        cloud_out.height = 1
        cloud_out.width = len(transformed_points)
        cloud_out.fields = PointCloud2.get_fields_list(transformed_points)
        cloud_out.point_step = PointCloud2.get_point_step(transformed_points)
        cloud_out.row_step = cloud_out.point_step * len(transformed_points)
        cloud_out.data = PointCloud2.serialize(transformed_points)

        # Publicar la nube de puntos en el tópico cloud
        self.cloud_publisher.publish(cloud_out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanToPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()