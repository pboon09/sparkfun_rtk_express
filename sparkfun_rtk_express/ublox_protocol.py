#!/usr/bin/env python3
# ublox_protocol.py - Library for UBX protocol communication with u-blox GPS modules

import serial
import struct
import time

class UbxProtocol:
    """Class for handling UBX protocol communication with u-blox GPS devices"""
    
    # UBX protocol constants
    UBX_HEADER = bytes([0xB5, 0x62])
    
    # Common UBX class IDs
    CLASS_NAV = 0x01  # Navigation Results
    CLASS_RXM = 0x02  # Receiver Manager Messages
    CLASS_INF = 0x04  # Information Messages
    CLASS_ACK = 0x05  # Acknowledgement Messages
    CLASS_CFG = 0x06  # Configuration Input Messages
    CLASS_MON = 0x0A  # Monitoring Messages
    CLASS_AID = 0x0B  # AssistNow Aiding Messages
    CLASS_TIM = 0x0D  # Timing Messages
    CLASS_ESF = 0x10  # External Sensor Fusion Messages
    CLASS_MGA = 0x13  # Multiple GNSS Assistance Messages
    CLASS_LOG = 0x21  # Logging Messages
    CLASS_SEC = 0x27  # Security Messages
    
    # Common message IDs
    # NAV class messages
    NAV_PVT = 0x07    # Navigation Position Velocity Time Solution
    NAV_DOP = 0x04    # Dilution of Precision
    NAV_SAT = 0x35    # Satellite Information
    NAV_STATUS = 0x03 # Receiver Navigation Status
    NAV_HPPOSLLH = 0x14 # High Precision Position
    
    # CFG class messages
    CFG_MSG = 0x01    # Set Message Rate
    CFG_RATE = 0x08   # Navigation/Measurement Rate Settings
    CFG_PRT = 0x00    # Port Configuration
    
    # MON class messages
    MON_VER = 0x04    # Receiver/Software Version
    MON_HW = 0x09     # Hardware Status
    MON_SYS = 0x39    # System Status
    
    # ACK class messages
    ACK_ACK = 0x01    # Message Acknowledged
    ACK_NAK = 0x00    # Message Not Acknowledged
    
    def __init__(self, port=None, baudrate=57600, timeout=1.0):
        """
        Initialize UBX protocol handler.
        
        Args:
            port: Serial port name (e.g., 'COM4'), or an already open serial.Serial object
            baudrate: Baud rate for serial communication (default: 57600)
            timeout: Serial read timeout in seconds (default: 1.0)
        """
        if port is None:
            self.serial = None
        elif isinstance(port, serial.Serial):
            self.serial = port
        else:
            self.serial = serial.Serial(port, baudrate, timeout=timeout)
        
        self.last_error = None
    
    def open(self, port, baudrate=57600, timeout=1.0):
        """
        Open a serial connection to a u-blox device.
        
        Args:
            port: Serial port name (e.g., 'COM4')
            baudrate: Baud rate for serial communication (default: 57600)
            timeout: Serial read timeout in seconds (default: 1.0)
            
        Returns:
            True if successful, False otherwise.
        """
        try:
            self.serial = serial.Serial(port, baudrate, timeout=timeout)
            return True
        except Exception as e:
            self.last_error = str(e)
            return False
    
    def close(self):
        """Close the serial connection."""
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
    
    def _calculate_checksum(self, message):
        """
        Calculate Fletcher checksum for a UBX message.
        
        Args:
            message: The message bytes (excluding header and checksum)
            
        Returns:
            Checksum as a bytes object.
        """
        ck_a = 0
        ck_b = 0
        for b in message:
            ck_a = (ck_a + b) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        return bytes([ck_a, ck_b])
    
    def send_message(self, msg_class, msg_id, payload=None, wait_for_ack=False, timeout=1.0):
        """
        Send a UBX message to the GPS device.
        
        Args:
            msg_class: UBX message class (1 byte)
            msg_id: UBX message ID (1 byte)
            payload: Optional payload bytes or list of integers
            wait_for_ack: Whether to wait for ACK/NAK response (default: False)
            timeout: Timeout in seconds when waiting for response (default: 1.0)
            
        Returns:
            If wait_for_ack is True:
                True if ACK received
                False if NAK received or timeout
            Otherwise:
                True if message was sent
        """
        if self.serial is None or not self.serial.is_open:
            return False
        
        # Prepare payload
        if payload is None:
            payload = b''
        elif isinstance(payload, list):
            payload = bytes(payload)
        elif not isinstance(payload, bytes):
            payload = bytes([payload])
        
        # Create message: class + id + length + payload
        length = len(payload)
        message = bytes([msg_class, msg_id]) + struct.pack('<H', length) + payload
        
        # Calculate checksum
        checksum = self._calculate_checksum(message)
        
        # Complete message with header and checksum
        complete_message = UbxProtocol.UBX_HEADER + message + checksum
        
        # Send the message
        self.serial.write(complete_message)
        
        # If no acknowledgment required, return
        if not wait_for_ack:
            return True
        
        # Wait for ACK/NAK
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            response = self.read_message(timeout=timeout)
            if response and response['class'] == UbxProtocol.CLASS_ACK:
                if response['id'] == UbxProtocol.ACK_ACK:
                    # ACK received - check if it's for our message
                    ack_payload = response['payload']
                    if len(ack_payload) >= 2 and ack_payload[0] == msg_class and ack_payload[1] == msg_id:
                        return True
                elif response['id'] == UbxProtocol.ACK_NAK:
                    # NAK received - check if it's for our message
                    nak_payload = response['payload']
                    if len(nak_payload) >= 2 and nak_payload[0] == msg_class and nak_payload[1] == msg_id:
                        return False
            
            # Give a small sleep to avoid hogging CPU
            time.sleep(0.01)
        
        # Timeout waiting for ACK/NAK
        return False
    
    def poll_message(self, msg_class, msg_id, payload=None, timeout=1.0):
        """
        Send a poll request and wait for the response.
        
        Args:
            msg_class: UBX message class (1 byte)
            msg_id: UBX message ID (1 byte)
            payload: Optional payload for poll request
            timeout: Timeout in seconds (default: 1.0)
            
        Returns:
            Dictionary with the response message or None if no response received.
        """
        if self.serial is None or not self.serial.is_open:
            return None
        
        # Send the poll request
        self.send_message(msg_class, msg_id, payload)
        
        # Wait for the response with matching class and ID
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            response = self.read_message(timeout=timeout)
            if response and response['class'] == msg_class and response['id'] == msg_id:
                return response
            
            # Give a small sleep to avoid hogging CPU
            time.sleep(0.01)
        
        # Timeout without receiving response
        return None
    
    def read_message(self, timeout=1.0):
        """
        Read a UBX message from the GPS device.
        
        Args:
            timeout: Timeout in seconds (default: 1.0)
            
        Returns:
            Dictionary with message class, ID, and payload, or None if no message received
        """
        if self.serial is None or not self.serial.is_open:
            return None
        
        # Timeout handling
        self.serial.timeout = timeout
        
        # Buffer to collect message bytes
        buffer = bytearray()
        
        # Keep track of time for timeout
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            # Read available bytes
            if self.serial.in_waiting > 0:
                buffer += self.serial.read(self.serial.in_waiting)
                
                # Look for UBX header
                idx = buffer.find(UbxProtocol.UBX_HEADER)
                if idx != -1:
                    # Remove bytes before header
                    if idx > 0:
                        buffer = buffer[idx:]
                    
                    # Check if we have enough bytes for a minimal UBX message
                    if len(buffer) < 8:  # Header(2) + Class(1) + ID(1) + Length(2) + Checksum(2)
                        continue
                    
                    # Extract message class, ID and length
                    msg_class = buffer[2]
                    msg_id = buffer[3]
                    length = buffer[4] + (buffer[5] << 8)  # Little-endian length
                    
                    # Check if we have the complete message
                    msg_end = 8 + length  # Header(2) + Class(1) + ID(1) + Length(2) + Payload(length) + Checksum(2)
                    if len(buffer) < msg_end:
                        continue
                    
                    # Extract payload and checksum
                    payload = buffer[6:6+length]
                    checksum = buffer[6+length:8+length]
                    
                    # Verify checksum
                    message = buffer[2:6+length]  # Class + ID + Length + Payload
                    calculated_checksum = self._calculate_checksum(message)
                    
                    if checksum == calculated_checksum:
                        # Remove processed message from buffer
                        buffer = buffer[8+length:]
                        
                        # Return parsed message
                        return {
                            'class': msg_class,
                            'id': msg_id,
                            'payload': payload
                        }
                    else:
                        # Bad checksum, remove header bytes and try again
                        buffer = buffer[2:]
            
            # Small delay to prevent CPU hogging
            time.sleep(0.01)
        
        # Timeout without finding valid message
        return None
    
    # ---------- Helper Methods for Common UBX Messages ----------
    
    def configure_message_rate(self, msg_class, msg_id, rate=1):
        """
        Configure how often a specific message should be sent.
        
        Args:
            msg_class: UBX message class
            msg_id: UBX message ID
            rate: Message rate (0=off, 1=every epoch, 2=every other epoch, etc.)
            
        Returns:
            True if configuration was successful, False otherwise
        """
        payload = bytes([msg_class, msg_id, rate])
        return self.send_message(UbxProtocol.CLASS_CFG, UbxProtocol.CFG_MSG, payload, wait_for_ack=True)
    
    def configure_navigation_rate(self, measurement_rate_ms=1000, nav_rate=1, time_ref=0):
        """
        Configure navigation/measurement rate.
        
        Args:
            measurement_rate_ms: Measurement rate in milliseconds
            nav_rate: Number of measurement cycles per navigation solution
            time_ref: Time reference (0=UTC, 1=GPS)
            
        Returns:
            True if configuration was successful, False otherwise
        """
        payload = struct.pack('<HHH', measurement_rate_ms, nav_rate, time_ref)
        return self.send_message(UbxProtocol.CLASS_CFG, UbxProtocol.CFG_RATE, payload, wait_for_ack=True)
    
    def get_position_data(self):
        """
        Get position, velocity, and time data (UBX-NAV-PVT).
        
        Returns:
            Dictionary with parsed NAV-PVT data or None if no response
        """
        response = self.poll_message(UbxProtocol.CLASS_NAV, UbxProtocol.NAV_PVT)
        
        if response:
            payload = response['payload']
            if len(payload) >= 92:  # NAV-PVT is 92 bytes
                result = {
                    'iTOW': int.from_bytes(payload[0:4], byteorder='little'),
                    'year': int.from_bytes(payload[4:6], byteorder='little'),
                    'month': payload[6],
                    'day': payload[7],
                    'hour': payload[8],
                    'min': payload[9],
                    'sec': payload[10],
                    'valid': payload[11],
                    'fixType': payload[20],
                    'numSV': payload[23],
                    'lon': int.from_bytes(payload[24:28], byteorder='little', signed=True) * 1e-7,
                    'lat': int.from_bytes(payload[28:32], byteorder='little', signed=True) * 1e-7,
                    'height': int.from_bytes(payload[32:36], byteorder='little', signed=True) * 1e-3,
                    'hMSL': int.from_bytes(payload[36:40], byteorder='little', signed=True) * 1e-3,
                    'hAcc': int.from_bytes(payload[40:44], byteorder='little') * 1e-3,
                    'vAcc': int.from_bytes(payload[44:48], byteorder='little') * 1e-3,
                    'velN': int.from_bytes(payload[48:52], byteorder='little', signed=True) * 1e-3,
                    'velE': int.from_bytes(payload[52:56], byteorder='little', signed=True) * 1e-3,
                    'velD': int.from_bytes(payload[56:60], byteorder='little', signed=True) * 1e-3,
                    'gSpeed': int.from_bytes(payload[60:64], byteorder='little', signed=True) * 1e-3,
                    'headMot': int.from_bytes(payload[64:68], byteorder='little', signed=True) * 1e-5,
                    'pDOP': int.from_bytes(payload[76:78], byteorder='little') * 0.01,
                }
                return result
        
        return None
    
    def get_system_status(self):
        """
        Get system status information (UBX-MON-SYS).
        
        Returns:
            Dictionary with parsed MON-SYS data or None if no response
        """
        response = self.poll_message(UbxProtocol.CLASS_MON, UbxProtocol.MON_SYS)
        
        if response:
            payload = response['payload']
            if len(payload) >= 24:
                result = {
                    'msgVer': payload[0],
                    'bootType': payload[1],
                    'cpuLoad': payload[2],
                    'cpuLoadMax': payload[3],
                    'memUsage': payload[4],
                    'memUsageMax': payload[5],
                    'ioUsage': payload[6],
                    'ioUsageMax': payload[7],
                    'runTime': int.from_bytes(payload[8:12], byteorder='little'),
                    'noticeCount': int.from_bytes(payload[12:14], byteorder='little'),
                    'warnCount': int.from_bytes(payload[14:16], byteorder='little'),
                    'errorCount': int.from_bytes(payload[16:18], byteorder='little'),
                    'tempValue': int.from_bytes(payload[18:19], byteorder='little', signed=True),
                }
                return result
        
        return None
    
    def get_receiver_version(self):
        """
        Get receiver and software version information (UBX-MON-VER).
        
        Returns:
            Dictionary with software version, hardware version, and extensions
        """
        response = self.poll_message(UbxProtocol.CLASS_MON, UbxProtocol.MON_VER)
        
        if response:
            payload = response['payload']
            if len(payload) >= 40:
                # Extract software version (30 bytes, null-terminated string)
                sw_version = payload[0:30].split(b'\x00')[0].decode('ascii')
                
                # Extract hardware version (10 bytes, null-terminated string)
                hw_version = payload[30:40].split(b'\x00')[0].decode('ascii')
                
                # Extract extension strings (each 30 bytes, null-terminated string)
                extensions = []
                for i in range(40, len(payload), 30):
                    if i + 30 <= len(payload):
                        ext_str = payload[i:i+30].split(b'\x00')[0].decode('ascii')
                        if ext_str:  # Only add non-empty strings
                            extensions.append(ext_str)
                
                return {
                    'sw_version': sw_version,
                    'hw_version': hw_version,
                    'extensions': extensions
                }
        
        return None
    
    def configure_for_high_rate(self, rate_ms=50):
        """
        Configure the GPS for high-rate position updates.
        
        Args:
            rate_ms: Measurement rate in milliseconds (default: 50ms = 20Hz)
            
        Returns:
            Dictionary with configuration results (success/failure)
        """
        results = {}
        
        # Set measurement rate
        results['nav_rate'] = self.configure_navigation_rate(rate_ms)
        
        # Enable NAV-PVT messages at every navigation solution
        results['nav_pvt'] = self.configure_message_rate(UbxProtocol.CLASS_NAV, UbxProtocol.NAV_PVT, 1)
        
        # Disable unnecessary NMEA messages to reduce bandwidth
        nmea_classes = [0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0]
        nmea_ids = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05]  # GGA, GLL, GSA, GSV, RMC, VTG
        
        results['nmea'] = True
        for i in range(len(nmea_classes)):
            if not self.configure_message_rate(nmea_classes[i], nmea_ids[i], 0):
                results['nmea'] = False
        
        return results

# Main function to execute when script is run directly
if __name__ == "__main__":
    import argparse
    
    # Default settings - change these as needed
    DEFAULT_PORT = "/dev/ttyACM0"  # Change to your GPS port
    DEFAULT_BAUDRATE = 57600
    DEFAULT_RATE = 50  # 50ms = 20Hz
    
    try:
        # Create GPS object
        gps = UbxProtocol()
        
        # Open connection
        print(f"Opening connection to GPS on {DEFAULT_PORT} at {DEFAULT_BAUDRATE} baud...")
        if not gps.open(DEFAULT_PORT, DEFAULT_BAUDRATE):
            print(f"Error opening serial port: {gps.last_error}")
            exit(1)

        print("\nReading position data. Press Ctrl+C to stop.")
        print("-" * 60)
        
        # Variables for rate calculation
        count = 0
        start_time = time.time()
        
        # Main loop
        while True:
            # Get position data
            position = gps.get_position_data()
            
            if position:
                count += 1
                current_time = time.time()
                rate = count / (current_time - start_time)
                
                # Print position data
                print(f"Rate: {rate:.2f}Hz | Latitude: {position['lat']:.7f}° | Longitude: {position['lon']:.7f}°")
                
                # Reset counter every 5 seconds to show current rate
                if (current_time - start_time) > 5:
                    print(f"\nAverage rate over 5 seconds: {rate:.2f}Hz")
                    count = 0
                    start_time = current_time
            
            # Small delay to prevent CPU hogging
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        print("\nProgram stopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Always close the connection
        gps.close()
        print("GPS connection closed")