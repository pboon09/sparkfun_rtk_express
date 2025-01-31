# SparkFunRTKExpress

## Overview
This package is designed to work with the SparkFun RTK Express GNSS receiver. It provides a [ROS2 interface for reading GNSS data](https://github.com/CARVER-NEXT-GEN/gps_interface), processing NMEA sentences, and publishing structured GPS messages.

## Setup your SparkFun
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

## Config Sparkfun with U-Center
1. Download U-Center
- Navigate to [U-Center Download Page](https://www.u-blox.com/en/product/u-center)
- Verify your u-blox chip number before download U-Center
2. Connect SparkFun
- Power On the SparkFun
- Connect your board at a specific port labeled “ConfigESP32” with a USB-C cable
- Look for the COM port (Windows) in Device Manager or the /dev/tty* device on Linux/macOS.
- Check U-Center’s message windows to ensure you see incoming GNSS data (NMEA or UBX messages).
- Use View > Messages View to modify specific settings like update rates, RTCM messages, or power modes.
- To save settings, go to View > Configuration View, select Save to Flash and BBR (battery-backed RAM).

## Cloning and Setting Up the Package
1. Clone the custom message package 
```
cd ~/ros2_ws/src
git clone https://github.com/CARVER-NEXT-GEN/gps_interface.git
```
2. Clone SparkFunRTKExpress package 
```
cd ~/ros2_ws/src
git clone https://github.com/CARVER-NEXT-GEN/sparkfun_rtk_express.git
```
3. Build the Package After cloning, build the package with
```
cd ~/ros2_ws
colcon build
```

## How to use
Once your RTK Express is connected and your ROS 2 environment is sourced, run the following command:
```
ros2 run sparkfun_rtk_express GPSRTKpublisher.py
```
This will launch a publisher node that reads NMEA (or UBX) data from the SparkFun RTK Express and publishes structured GPS messages.

You can confirm the output of the GNSS messages by echoing the topic:
```
ros2 topic list
ros2 topic echo /gps_data
```
If you want to see more example, check the `example.py`
