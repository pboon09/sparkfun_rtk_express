#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import time
import threading

# Import UBX protocol handler
from sparkfun_rtk_express.ublox_protocol import UbxProtocol

class UbxGpsPublisher(Node):
    def __init__(self):
        super().__init__('ubx_gps_publisher')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyRTKExpress')
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('high_rate', True)
        self.declare_parameter('measurement_rate_ms', 50)
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.rate_hz = self.get_parameter('rate_hz').value
        self.high_rate = self.get_parameter('high_rate').value
        self.measurement_rate_ms = self.get_parameter('measurement_rate_ms').value
        
        # Create publisher
        self.fix_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        
        # Initialize UBX GPS
        self.gps = UbxProtocol()
        self.is_running = False
        
        # Create a timer for publishing
        timer_period = 1.0 / self.rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Connect to GPS in a separate thread to avoid blocking node startup
        self.connect_thread = threading.Thread(target=self.connect_gps)
        self.connect_thread.daemon = True
        self.connect_thread.start()
        
        self.get_logger().info(f"UBX GPS Publisher started on {self.port} at {self.baudrate} baud, publishing at {self.rate_hz} Hz")
    
    def connect_gps(self):
        """Connect to GPS in a separate thread"""
        try:
            self.get_logger().info(f"Connecting to GPS on {self.port}...")
            
            if not self.gps.open(self.port, self.baudrate):
                self.get_logger().error(f"Failed to open port: {self.gps.last_error}")
                return
            
            self.get_logger().info("GPS connected successfully")

            self.get_logger().info(f"Configuring GPS for {1000/self.measurement_rate_ms:.1f}Hz rate...")
            results = self.gps.configure_for_high_rate(self.measurement_rate_ms)
            self.get_logger().info(f"Configuration results: {results}")

            
            self.is_running = True
            
        except Exception as e:
            self.get_logger().error(f"Error connecting to GPS: {e}")
    
    def timer_callback(self):
        """Timer callback to publish GPS data"""
        if not self.is_running:
            return
        
        try:
            # Get position data
            position = self.gps.get_position_data()
            
            if position:
                # Create NavSatFix message
                fix_msg = NavSatFix()
                fix_msg.header.stamp = self.get_clock().now().to_msg()
                fix_msg.header.frame_id = self.frame_id
                
                # Set position data
                fix_msg.latitude = position['lat']
                fix_msg.longitude = position['lon']
                fix_msg.altitude = position.get('hMSL', 0.0)  # Height above mean sea level
                
                # Set fix status based on fixType
                fix_type = position.get('fixType', 0)
                if fix_type >= 3:  # 3D fix or better
                    fix_msg.status.status = NavSatStatus.STATUS_FIX
                elif fix_type >= 2:  # 2D fix
                    fix_msg.status.status = NavSatStatus.STATUS_FIX
                else:
                    fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
                
                fix_msg.status.service = NavSatStatus.SERVICE_GPS
                
                # Set position covariance if available
                h_acc = position.get('hAcc', 0.0)  # Horizontal accuracy
                v_acc = position.get('vAcc', 0.0)  # Vertical accuracy
                
                if h_acc > 0.0:
                    # Convert accuracy to variance (square of standard deviation)
                    h_var = h_acc * h_acc
                    v_var = v_acc * v_acc
                    
                    # Set position covariance (ENU format)
                    fix_msg.position_covariance = [
                        h_var, 0.0, 0.0,  # East variance, cross-covariances
                        0.0, h_var, 0.0,  # North variance, cross-covariance
                        0.0, 0.0, v_var   # Up variance
                    ]
                    fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                else:
                    fix_msg.position_covariance = [0.0] * 9
                    fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                
                # Publish the message
                self.fix_pub.publish(fix_msg)
                self.get_logger().debug(f"Published NavSatFix: lat={fix_msg.latitude:.7f}, lon={fix_msg.longitude:.7f}")
            
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {e}")
    
    def destroy_node(self):
        """Clean up when node is shutting down"""
        self.get_logger().info("Shutting down GPS node")
        self.is_running = False
        
        if self.gps.serial is not None:
            self.gps.close()
            self.get_logger().info("GPS connection closed")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UbxGpsPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()