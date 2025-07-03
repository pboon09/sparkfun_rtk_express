#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from typing import Optional

# ROS2 message types
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GPSCheckDataNode(Node):
    def __init__(self):
        super().__init__('gps_check_data_node')
        
        # Parameters
        self.declare_parameter('display_rate', 5.0)  # Hz - how often to print data
        self.declare_parameter('precision', 7)  # decimal places
        
        self.display_rate = self.get_parameter('display_rate').get_parameter_value().double_value
        self.precision = self.get_parameter('precision').get_parameter_value().integer_value
        
        # Subscribe to GPS topic
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )
        
        # Timer for periodic display
        self.display_timer = self.create_timer(
            1.0 / self.display_rate,
            self.display_callback
        )
        
        # Store latest GPS data and tracking
        self.latest_gps_data: Optional[NavSatFix] = None
        self.data_received = False
        
        # Dynamic status tracking
        self.fix_history = []
        self.position_history = []
        self.max_history = 20
        
        # Statistics
        self.message_count = 0
        self.last_display_time = self.get_clock().now()
        
        self.get_logger().info("GPS Data Checker Node started")
        self.get_logger().info(f"Subscribing to /gps/fix topic")
        self.get_logger().info(f"Display rate: {self.display_rate} Hz")
        self.get_logger().info(f"Precision: {self.precision} decimal places")
        self.get_logger().info("=" * 80)
    
    def gps_callback(self, msg: NavSatFix):
        """Store the latest GPS message and update tracking"""
        self.latest_gps_data = msg
        self.data_received = True
        self.message_count += 1
        
        # Track fix status history
        self.fix_history.append(msg.status.status)
        if len(self.fix_history) > self.max_history:
            self.fix_history.pop(0)
        
        # Track position history for stability analysis
        if msg.status.status != NavSatStatus.STATUS_NO_FIX:
            self.position_history.append({
                'lat': msg.latitude,
                'lon': msg.longitude,
                'alt': msg.altitude
            })
            if len(self.position_history) > self.max_history:
                self.position_history.pop(0)
    
    def get_status_name(self, status_code: int) -> str:
        """Convert status code to readable name"""
        status_names = {
            NavSatStatus.STATUS_NO_FIX: "NO_FIX",
            NavSatStatus.STATUS_FIX: "GPS_FIX",
            NavSatStatus.STATUS_SBAS_FIX: "RTK_FLOAT",
            NavSatStatus.STATUS_GBAS_FIX: "RTK_FIXED"
        }
        return status_names.get(status_code, f"UNKNOWN({status_code})")
    
    def get_service_name(self, service_code: int) -> str:
        """Convert service code to readable name"""
        service_names = {
            NavSatStatus.SERVICE_GPS: "GPS",
            NavSatStatus.SERVICE_GLONASS: "GLONASS",
            NavSatStatus.SERVICE_COMPASS: "COMPASS",
            NavSatStatus.SERVICE_GALILEO: "GALILEO"
        }
        return service_names.get(service_code, f"UNKNOWN({service_code})")
    
    def get_covariance_type_name(self, cov_type: int) -> str:
        """Convert covariance type to readable name"""
        cov_type_names = {
            NavSatFix.COVARIANCE_TYPE_UNKNOWN: "UNKNOWN",
            NavSatFix.COVARIANCE_TYPE_APPROXIMATED: "APPROXIMATED",
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN: "DIAGONAL_KNOWN",
            NavSatFix.COVARIANCE_TYPE_KNOWN: "KNOWN"
        }
        return cov_type_names.get(cov_type, f"UNKNOWN({cov_type})")
    
    def extract_covariance_diagonal(self, cov_matrix: list) -> list:
        """Extract diagonal elements from 3x3 covariance matrix (flattened)"""
        if len(cov_matrix) == 9:
            return [cov_matrix[0], cov_matrix[4], cov_matrix[8]]  # East, North, Up
        return [0.0, 0.0, 0.0]
    
    def display_callback(self):
        """Display GPS data periodically"""
        if not self.data_received or self.latest_gps_data is None:
            self.get_logger().warn("No GPS data received yet...")
            return
        
        msg = self.latest_gps_data
        
        # Calculate data rate
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_display_time).nanoseconds / 1e9
        data_rate = self.message_count / time_diff if time_diff > 0 else 0.0
        
        # Extract covariance diagonal elements
        position_cov = self.extract_covariance_diagonal(msg.position_covariance)
        
        # Format numbers with specified precision
        p = self.precision
        
        # Display header
        print("\n" + "=" * 80)
        print(f"GPS DATA CHECK - Messages: {self.message_count}, Rate: {data_rate:.1f} Hz")
        print(f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}")
        print(f"Frame ID: {msg.header.frame_id}")
        print("-" * 80)
        
        # Position
        print(f"POSITION (WGS84):")
        print(f"  Latitude:  {msg.latitude:+{p+10}.{p}f}°")
        print(f"  Longitude: {msg.longitude:+{p+10}.{p}f}°")
        print(f"  Altitude:  {msg.altitude:+{p+7}.3f} m")
        
        print()
        
        # Status
        status_name = self.get_status_name(msg.status.status)
        service_name = self.get_service_name(msg.status.service)
        print(f"STATUS:")
        print(f"  Fix Status: {status_name}")
        print(f"  Service:    {service_name}")
        
        print()
        
        # Covariance
        cov_type_name = self.get_covariance_type_name(msg.position_covariance_type)
        print(f"COVARIANCE:")
        print(f"  Type: {cov_type_name}")
        print(f"  East (X):  {position_cov[0]:.{p}f}")
        print(f"  North (Y): {position_cov[1]:.{p}f}")
        print(f"  Up (Z):    {position_cov[2]:.{p}f}")
        
        print()
        
        # Accuracy (from covariance)
        h_acc_east = math.sqrt(position_cov[0]) if position_cov[0] > 0 else 0.0
        h_acc_north = math.sqrt(position_cov[1]) if position_cov[1] > 0 else 0.0
        v_acc_up = math.sqrt(position_cov[2]) if position_cov[2] > 0 else 0.0
        
        print(f"ACCURACY (m):")
        print(f"  Horizontal East:  {h_acc_east:.3f}")
        print(f"  Horizontal North: {h_acc_north:.3f}")
        print(f"  Vertical Up:      {v_acc_up:.3f}")
        
        print()
        
        # Dynamic Status Analysis
        print(f"DYNAMIC STATUS:")
        
        # Fix stability analysis
        if len(self.fix_history) >= 5:
            recent_fixes = self.fix_history[-5:]
            unique_fixes = set(recent_fixes)
            if len(unique_fixes) == 1:
                print(f"  ✓ Fix STABLE (last 5: {self.get_status_name(recent_fixes[0])})")
            else:
                fix_names = [self.get_status_name(f) for f in recent_fixes]
                print(f"  ⚠ Fix CHANGING (last 5: {' → '.join(fix_names)})")
        
        # Position stability analysis
        if len(self.position_history) >= 5:
            positions = np.array([[p['lat'], p['lon'], p['alt']] for p in self.position_history])
            
            # Calculate standard deviation
            std_lat = np.std(positions[:, 0])
            std_lon = np.std(positions[:, 1])
            std_alt = np.std(positions[:, 2])
            
            # Convert to meters (rough approximation)
            std_horizontal_m = np.sqrt((std_lat * 111000)**2 + (std_lon * 111000)**2)
            
            if std_horizontal_m < 0.5:
                print(f"  ✓ Position STABLE (σ={std_horizontal_m:.2f}m)")
            elif std_horizontal_m < 2.0:
                print(f"  ⚠ Position DRIFTING (σ={std_horizontal_m:.2f}m)")
            else:
                print(f"  ✗ Position UNSTABLE (σ={std_horizontal_m:.2f}m)")
        
        # Fix progression trend
        if len(self.fix_history) >= 10:
            rtk_fixed_count = self.fix_history.count(NavSatStatus.STATUS_GBAS_FIX)
            rtk_float_count = self.fix_history.count(NavSatStatus.STATUS_SBAS_FIX)
            gps_fix_count = self.fix_history.count(NavSatStatus.STATUS_FIX)
            no_fix_count = self.fix_history.count(NavSatStatus.STATUS_NO_FIX)
            
            total = len(self.fix_history)
            print(f"  Fix distribution (last {total}): RTK_FIXED={rtk_fixed_count}, RTK_FLOAT={rtk_float_count}, GPS={gps_fix_count}, NO_FIX={no_fix_count}")
            
            if rtk_fixed_count > total * 0.8:
                print(f"  ✓ EXCELLENT RTK performance ({rtk_fixed_count}/{total} fixed)")
            elif rtk_fixed_count + rtk_float_count > total * 0.7:
                print(f"  ✓ GOOD RTK performance ({rtk_fixed_count + rtk_float_count}/{total} RTK)")
            elif gps_fix_count > total * 0.7:
                print(f"  ⚠ BASIC GPS performance (no RTK)")
            else:
                print(f"  ✗ POOR performance (frequent fix loss)")
        
        print()
        
        # Status indicators
        print(f"STATUS INDICATORS:")
        if msg.status.status == NavSatStatus.STATUS_GBAS_FIX:
            print(f"  ✓ RTK FIXED solution")
        elif msg.status.status == NavSatStatus.STATUS_SBAS_FIX:
            print(f"  ✓ RTK FLOAT solution")
        elif msg.status.status == NavSatStatus.STATUS_FIX:
            print(f"  ✓ Standard GPS fix")
        else:
            print(f"  ✗ No GPS fix")
        
        # Rate check
        if 4.0 <= data_rate <= 6.0:
            print(f"  ✓ Good message rate ({data_rate:.1f} Hz)")
        else:
            print(f"  ⚠ Unexpected rate ({data_rate:.1f} Hz, expected ~5Hz)")
        
        print("=" * 80)
        
        # Reset counters
        self.message_count = 0
        self.last_display_time = current_time
    
    def destroy_node(self):
        self.get_logger().info("GPS Data Checker Node shutting down")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        gps_check_node = GPSCheckDataNode()
        rclpy.spin(gps_check_node)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down...")

    except Exception as e:
        print(f"Error in main: {e}")
        
    finally:
        if 'gps_check_node' in locals():
            gps_check_node.destroy_node()
        
        rclpy.shutdown()
        print("GPS Data Checker Node shutdown complete")

if __name__ == '__main__':
    main()