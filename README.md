# SparkFunRTKExpress
This package is designed to work with the [SparkFun RTK Express Kit](https://www.sparkfun.com/sparkfun-rtk-express-kit.html). It provides a ROS2 Node that publishes NavSatFix messages, launch files to visualize the data using mapviz, and a UBX protocol implementation to communicate with the u-blox F9P GNSS module.

## Features
- Direct communication with u-blox F9P GNSS module using UBX protocol
- ROS2 publisher for standard NavSatFix messages
- Configurable update rates up to 20Hz
- Mapviz integration for real-time position visualization
- Support for RTK corrections for high-precision positioning

## Dependencies
- ROS2 (tested on Humble)
- Python 3.8+
- pyserial
- mapviz (for visualization)
- swri_transform_util (for coordinate transformations)

## Setup Base and Rover
To configure your RTK base station and rover for centimeter-level positioning accuracy, please follow [the SparkFun RTK Setup Guide](https://learn.sparkfun.com/tutorials/setting-up-a-rover-base-rtk-system). This guide provides step-by-step instructions for establishing proper communication between base station and rover.

For transmitting RTCM correction data between base and rover:
- You will need a way to transmit the RTCM correction data from the base to the rover
- Options include: computer with internet connection, serial point to point radio, XBee modules, etc.
- The SparkFun guide recommends using a 915MHz Serial Telemetry Radio kit
- For this implementation, we used the Holybro V3 3DR Radio 433 Telemetry 500mW 433MHz Data System (compatible with APM2.6 Pixhawk PX4 controllers)

## Download Mapviz
For visualization of GPS data, you'll need to install Mapviz. Please follow the [official Mapviz documentation](https://swri-robotics.github.io/mapviz/) for installation instructions. For additional usage information, the [Robotics Knowledge Base guide](https://roboticsknowledgebase.com/wiki/tools/mapviz/) provides helpful examples and configuration tips.

## Hardware Setup and Configuration
### Updating ESP32 Firmware
1. Download the Firmware Uploader
- Navigate to the [SparkFun RTK Firmware Uploader GitHub Repo](https://github.com/sparkfun/SparkFun_RTK_Firmware_Uploader).
- Download the latest release for your operating system (Windows, Mac, or Linux).
- Extract or install it according to the instructions on the repo page.
2. Download the Latest Firmware Release
- Navigate to the [SparkFun RTK Firmware Releases](https://github.com/sparkfun/SparkFun_RTK_Firmware).
- Download the firmware .bin file (e.g., u-blox_F9P_SparkFun_RTK.x.x.x.bin) you - wish to flash onto your SparkFun RTK Express.
3. Flash the Firmware
- Power On the SparkFun
- Connect your board at a specific port labeled “ConfigESP32” with a USB-C cable
- Keep track of the location where you saved the .bin file
- Follow the instructions from Uploader Github Repo.

### Updating u-blox GNSS Firmware
1. Download the Latest u-blox Firmware
- Go to the [ZED-F9P Module Documentation](https://www.u-blox.com/en/product/zed-f9p-module?legacy=Current#Documentation-&-resources)
- Under "Firmware Update", select and download the firmware version you want
2. Install u-center and Update GNSS Firmware
- Download [u-center](https://www.u-blox.com/en/product/u-center) (verify your u-blox chip model first)
- Connect your RTK Express to your computer
- Use u-center to flash the new firmware following the u-blox documentation

### Configuring SparkFun RTK Express with u-center
1. Set Up u-center Connection
- Power on the SparkFun RTK Express
- Connect using the appropriate USB port
- Identify the COM port (Windows) or /dev/tty* device (Linux/macOS)
2. Configure Settings
- Check u-center's message windows to verify you're receiving GNSS data (NMEA or UBX messages)
- Use View > Messages View to modify specific settings (update rates, RTCM messages, power modes)
- To save settings permanently, go to View > Configuration View, select "Save to Flash and BBR"

## Protocol Message
The `ublox_protocol.py` module in this package is based on the official UBX protocol specifications from u-blox:
1. Get the Interface Manual
- Go to the [ZED-F9P Module Documentation](https://www.u-blox.com/en/product/zed-f9p-module?legacy=Current#Documentation-&-resources)
- Under "Interface Manual", select the manual corresponding to your firmware version
- This manual contains detailed information about all UBX protocol messages
2. Message Reference
- All messages implemented in this package follow the official UBX protocol specification
- When sending messages to the device, message structures and formats adhere to the interface manual
- The protocol module handles checksum calculation, message framing, and response parsing

## Instalation
### Installing Python Dependencies
```bash
chmod +x install_python_deps.sh
./install_python_deps.sh
```

### Building the Package
```bash
mkdir -p ~/sparkfun_ws/src
cd ~/sparkfun_ws/src

git clone https://github.com/pboon09/sparkfun_rtk_express.git

cd ~/sparkfun_ws
colcon build --symlink-install --packages-select sparkfun_rtk_express

source ~/sparkfun_ws/install/setup.bash
```

## Usage
### Basic Usage
To start the GPS publisher node:
```bash
ros2 run sparkfun_rtk_express ubx_gps_publisher.py --ros-args -p port:=/dev/ttyACM0
```
This will publish GNSS data to the `/gps/fix` topic as NavSatFix messages.
### Parameters
The `ubx_gps_publisher` node supports the following parameters:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| port | String | COM4 | Serial port of the GPS device |
| baudrate | Integer | 57600 | Serial baudrate |
| frame_id | String | gps_link | TF frame ID for GPS measurements |
| rate_hz | Float | 12.5 | Publishing rate in Hz |
| high_rate | Boolean | True | Enable high-rate mode |
| measurement_rate_ms | Integer | 80 | GPS measurement rate in milliseconds |
```bash
ros2 run sparkfun_rtk_express ubx_gps_publisher.py --ros-args \
  -p port:=/dev/ttyACM0 \
  -p baudrate:=115200 \
  -p rate_hz:=20.0 \
  -p measurement_rate_ms:=50
```
### Using with MapViz
The package includes a launch file to start MapViz with a pre-configured setup for visualizing GPS data:
```bash
ros2 launch sparkfun_rtk_express mapviz_launch.launch.py
```
This will launch:
- MapViz with a map display and a NavSat display for GPS visualization
- Origin initialization for local coordinate frame
- Static transform publisher for the map frame

## Feedback
If you have any feedback, please create an issue and I will answer your questions there.