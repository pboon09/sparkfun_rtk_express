#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gps_interface.msg import GNSSData
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GpsToNavSatFix(Node):
    def __init__(self):
        super().__init__('gps_to_navsatfix')

        self.declare_parameter('frame_id', 'gps_link')

        self.subscription = self.create_subscription(
            GNSSData,
            'gps_data',              #
            self.gps_callback,
            10
        )
        
        self.fix_pub = self.create_publisher(NavSatFix, 'fix', 10)
        
        self.get_logger().info("gps_to_navsatfix node started")

    def gps_callback(self, gnss_msg):
        fix_msg = NavSatFix()
        fix_msg.header.stamp = self.get_clock().now().to_msg()
        fix_msg.header.frame_id = self.get_parameter('frame_id').value

        fix_msg.latitude = gnss_msg.latitude
        fix_msg.longitude = gnss_msg.longitude
        fix_msg.altitude = 0.0

        fix_msg.status.status = NavSatStatus.STATUS_FIX
        fix_msg.status.service = NavSatStatus.SERVICE_GPS

        fix_msg.position_covariance = [0.0] * 9
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.fix_pub.publish(fix_msg)
        self.get_logger().debug(f"Published NavSatFix: lat={fix_msg.latitude}, lon={fix_msg.longitude}")


def main(args=None):
    rclpy.init(args=args)
    node = GpsToNavSatFix()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
