#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sparkfun_rtk_express.GNSSReceiver import GNSSReceiver
from gps_interface.msg import GNSSData 


class VehicleGpsPublisher(Node):
    def __init__(self):
        super().__init__('vehicle_gps_publisher')

        self.receiver = GNSSReceiver(port="/dev/ttyACM1", baudrate=230400, timeout=0.1)
        self.gps_pub = self.create_publisher(GNSSData, 'gps_data', 10)

        self.get_logger().info("vehicle_gps_publisher node started")
        self.timer = self.create_timer(0.1, self.publish_data)  # 10Hz

    def publish_data(self):
        self.receiver.read_once()

        gps_msg = GNSSData()

        # === GGA (Fix Data) ===
        gga = self.receiver.get_latest_data("GGA")
        if gga:
            gps_msg.latitude = gga.get('latitude', 0.0)
            gps_msg.longitude = gga.get('longitude', 0.0)
            gps_msg.altitude = gga.get('altitude', 0.0)
            gps_msg.num_sats = gga.get('num_sats', 0)
            gps_msg.fix_status = gga.get('gps_qual', 0)
            gps_msg.hdop = gga.get('hdop', 0.0)

        # === RMC (Recommended Minimum Data) ===
        rmc = self.receiver.get_latest_data("RMC")
        if rmc:
            gps_msg.speed_knots = rmc.get('speed_knots', 0.0)
            gps_msg.speed_mps = gps_msg.speed_knots * 0.514444  # Convert knots to m/s
            gps_msg.true_course = rmc.get('true_course', 0.0)
            gps_msg.status = rmc.get('status', 'V')
            gps_msg.datetime = str(rmc.get('datetime', ''))

        # === GLL (Geographic Position) ===
        gll = self.receiver.get_latest_data("GLL")
        if gll:
            gps_msg.gll_status = gll.get('status', 'V')

        # === GSA (DOP and Active Satellites) ===
        gsa = self.receiver.get_latest_data("GSA")
        if gsa:
            gps_msg.mode = gsa.get('mode', 'M')
            gps_msg.mode_fix_type = gsa.get('mode_fix_type', 1)
            gps_msg.pdop = gsa.get('pdop', 99.99)
            gps_msg.hdop_gsa = gsa.get('hdop', 99.99)
            gps_msg.vdop = gsa.get('vdop', 99.99)

        # === GSV (Satellites in View) ===
        gsv = self.receiver.get_latest_data("GSV")
        if gsv:
            gps_msg.num_sv_in_view = gsv.get('num_sv_in_view', 0)
            gps_msg.num_messages = gsv.get('num_messages', 0)
            gps_msg.msg_num = gsv.get('msg_num', 0)
            satellites = gsv.get('satellites', [])
            for i in range(min(4, len(satellites))):  # Max 4 per message
                gps_msg.sv_prn[i] = satellites[i].get('prn', 0)
                gps_msg.elevation[i] = satellites[i].get('elevation', 0.0)
                gps_msg.azimuth[i] = satellites[i].get('azimuth', 0.0)
                gps_msg.snr[i] = satellites[i].get('snr', 0.0)

        # === VTG (Course and Speed) ===
        vtg = self.receiver.get_latest_data("VTG")
        if vtg:
            gps_msg.course_true = vtg.get('course_true', 0.0)
            gps_msg.course_magnetic = vtg.get('course_magnetic', 0.0)
            gps_msg.spd_over_grnd_kts = vtg.get('spd_over_grnd_kts', 0.0)
            gps_msg.spd_over_grnd_kmph = vtg.get('spd_over_grnd_kmph', 0.0)

        # Store the last raw NMEA sentence
        gps_msg.last_raw_nmea = self.receiver.get_latest_raw() or ""

        # Publish the message
        self.gps_pub.publish(gps_msg)
        self.get_logger().info(f"Published GPS Data at {gps_msg.datetime}")

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
