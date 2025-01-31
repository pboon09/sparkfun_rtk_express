import time
import serial
import pynmea2

class GNSSReceiver:
    """
    A class to read and parse NMEA sentences from a GNSS/RTK receiver,
    store the latest parsed data, measure message rates, and provide raw data.
    """
    
    def __init__(self, port="/dev/ttyACM0", baudrate=230400, timeout=1):
        """
        Initialize the GNSSReceiver with serial port parameters.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        
        # Open the serial port
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        
        # Dictionary to store latest parsed data for each sentence type.
        self.data = {}
        
        # For measuring frequency (overall)
        self._msg_count_all = 0
        self._all_start_time = time.time()
        self._freq_all = 0.0  # last computed frequency (all sentences)
        
        # For measuring frequency (GGA only)
        self._msg_count_gga = 0
        self._gga_start_time = time.time()
        self._freq_gga = 0.0
        
        # For storing the last raw line we read (for debugging)
        self._last_raw_line = None
    
    def close(self):
        """Close the serial port when done."""
        if self.ser.is_open:
            self.ser.close()
    
    def read_forever(self):
        """
        Continuously read from the serial port, parse NMEA lines,
        store the latest data, and compute the message rates.
        This method blocks indefinitely (CTRL+C to stop).
        """
        while True:
            self.read_once()
    
    def read_once(self):
        """
        Read one NMEA line from the serial port, parse it, store the data,
        and update frequency measurements.
        Call this repeatedly if you want to control the loop yourself.
        """
        # Read a single line from serial
        line = self.ser.readline().decode('ascii', errors='replace').strip()
        
        # Store the raw line
        self._last_raw_line = line
        
        # Only process valid NMEA lines that start with '$'
        if not line.startswith('$'):
            return
        
        # Count every sentence (for overall frequency)
        self._msg_count_all += 1
        
        # Parse with pynmea2
        try:
            msg = pynmea2.parse(line)
        except pynmea2.ParseError:
            # Malformed sentence; just ignore
            return
        
        # Measure overall frequency (all sentences) ~once/sec
        elapsed_all = time.time() - self._all_start_time
        if elapsed_all >= 1.0:
            self._freq_all = self._msg_count_all / elapsed_all
            self._all_start_time = time.time()
            self._msg_count_all = 0
        
        # Identify the sentence type, e.g. "GGA", "RMC", "GSA", "GSV", "VTG", "GLL"...
        sentence_type = msg.sentence_type
        
        # If GGA, measure GGA frequency separately
        if sentence_type == "GGA":
            self._msg_count_gga += 1
            elapsed_gga = time.time() - self._gga_start_time
            if elapsed_gga >= 1.0:
                self._freq_gga = self._msg_count_gga / elapsed_gga
                self._gga_start_time = time.time()
                self._msg_count_gga = 0
            
            # Store GGA data
            self.data["GGA"] = {
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "altitude": msg.altitude,
                "num_sats": msg.num_sats,
                "gps_qual": msg.gps_qual,  # Fix type (0=no fix, 1=GPS, 2=DGPS, etc.)
                "timestamp": msg.timestamp
            }
        
        elif sentence_type == "RMC":
            # RMC includes date/time
            # pynmea2 tries to combine msg.datestamp + msg.timestamp into msg.datetime if both are valid
            # If not, msg.datetime might be None.
            # We'll store it for reference.
            parsed_dt = msg.datetime  # This is a datetime.datetime or None
            
            self.data["RMC"] = {
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "speed_knots": msg.spd_over_grnd,
                "true_course": msg.true_course,
                "datestamp": msg.datestamp,  # A date object or None
                "timestamp": msg.timestamp,   # A time object or None
                "datetime": parsed_dt,
                "status": msg.status  # 'A' = valid, 'V' = warning/no fix
            }
        
        elif sentence_type == "GLL":
            self.data["GLL"] = {
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "timestamp": msg.timestamp,
                "status": msg.status,
            }
        
        elif sentence_type == "GSA":
            self.data["GSA"] = {
                "mode": msg.mode,
                "mode_fix_type": msg.mode_fix_type,  # 1=No fix,2=2D,3=3D
                "pdop": msg.pdop,
                "hdop": msg.hdop,
                "vdop": msg.vdop,
            }
        
        elif sentence_type == "GSV":
            # GSV can report info on up to 4 satellites per line
            satellites = []
            for i in range(1, 5):
                prn = getattr(msg, f"sv_prn_{i}", None)
                elev = getattr(msg, f"elevation_deg_{i}", None)
                azm = getattr(msg, f"azimuth_deg_{i}", None)
                snr = getattr(msg, f"snr_{i}", None)
                if prn is not None:
                    satellites.append({
                        "prn": prn,
                        "elevation": elev,
                        "azimuth": azm,
                        "snr": snr
                    })
            
            self.data["GSV"] = {
                "num_sv_in_view": msg.num_sv_in_view,
                "msg_num": msg.msg_num,          # which message in the sequence
                "num_messages": msg.num_messages,
                "satellites": satellites
            }
        
        elif sentence_type == "VTG":
            self.data["VTG"] = {
                "course_true": msg.true_track,
                "course_magnetic": msg.mag_track,
                "spd_over_grnd_kts": msg.spd_over_grnd_kts,
                "spd_over_grnd_kmph": msg.spd_over_grnd_kmph,
            }
        
        else:
            # For unhandled sentence types, just store the raw string or entire parsed object
            self.data[sentence_type] = str(msg)
    
    def get_latest_data(self, sentence_type):
        """
        Retrieve the latest dictionary (or string) for a specific sentence type.
        Example usage: 'GGA', 'RMC', 'GSV', etc.
        Returns None if that sentence hasn't been seen yet.
        """
        return self.data.get(sentence_type, None)
    
    def get_frequency_all(self):
        """
        Returns the last computed overall frequency (messages per second)
        across all NMEA sentences, updated about once per second.
        """
        return self._freq_all
    
    def get_frequency_gga(self):
        """
        Returns the last computed frequency (messages per second)
        for GGA sentences only, updated about once per second.
        """
        return self._freq_gga
    
    def get_latest_raw(self):
        """
        Returns the last raw NMEA line read from the device (may or may not be valid).
        """
        return self._last_raw_line

if __name__ == "__main__":
    receiver = GNSSReceiver(port="/dev/ttyACM1", baudrate=230400, timeout=1)
    
    try:
        while True:
            receiver.read_once()  # Read + parse one line from serial
            
            # Print overall frequency and GGA frequency
            freq_all = receiver.get_frequency_all()
            freq_gga = receiver.get_frequency_gga()
            print(f"[{time.ctime()}] Overall rate: {freq_all:.1f} Hz | GGA rate: {freq_gga:.1f} Hz")
            
            # Show the last raw line (optional) - uncomment if you want to see it
            # print("Last raw line:", receiver.get_latest_raw())
            
            # Optionally print GGA data
            gga_data = receiver.get_latest_data("GGA")
            if gga_data:
                print(
                    f"  GGA fix: lat={gga_data['latitude']}, "
                    f"lon={gga_data['longitude']}, "
                    f"alt={gga_data['altitude']}, "
                    f"sat_count={gga_data['num_sats']}, "
                    f"fix_type={gga_data['gps_qual']}"
                )
            
            # Optionally show RMC date/time
            rmc_data = receiver.get_latest_data("RMC")
            if rmc_data:
                if rmc_data["datetime"]:
                    # This is the GNSS-based date/time
                    print(f"  RMC datetime (GNSS): {rmc_data['datetime']}")
                else:
                    print("  RMC datetime: None (invalid fix or no date/time)")

            # Slight delay to avoid printing too many lines per second
            time.sleep(0.1)

    except KeyboardInterrupt:
        receiver.close()
        print("Stopped by user.")
