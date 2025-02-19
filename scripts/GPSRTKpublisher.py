#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sparkfun_rtk_express.GNSSReceiver import GNSSReceiver
from gps_interface.msg import GNSSData 

class VehicleGpsPublisher(Node):
    def __init__(self):
        super().__init__('vehicle_gps_publisher')

        timer_period = 1/10.0

        self.receiver = GNSSReceiver(port="/dev/ttyACM0", baudrate=230400, timeout=0.1)

        self.gps_pub = self.create_publisher(GNSSData, 'gps_data', 10)

        self.get_logger().info("vehicle_gps_publisher node started")
        self.timer = self.create_timer(timer_period, self.publish_data)

    def publish_data(self):
        self.receiver.read_once()

        gga_data = self.receiver.get_latest_data("GGA")
        if not gga_data:
            self.get_logger().warn("No GGA data available yet.")
            return

        gps_msg = GNSSData()
        gps_msg.lon = gga_data['longitude']
        gps_msg.lat = gga_data['latitude']
        self.gps_pub.publish(gps_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleGpsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.receiver.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
