import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg

class LaserScanToPointCloudNode(Node):
    def __init__(self):
        super().__init__('laser_scan_to_point_cloud_node')
        self.laserProj = lg.LaserProjection()
        self.pcPub = self.create_publisher(PointCloud2, 'cloud', 10)
        self.laserSub = self.create_subscription(LaserScan, 'scan', self.scanCallback, 10)

    def scanCallback(self, msg):
        cloud = self.laserProj.projectLaser(msg)
        self.pcPub.publish(cloud)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToPointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
