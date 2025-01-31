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
- Power On the SparkFun with a USB-C cable
- Connect your board at a specific port labeled “ConfigESP32”
- Keep track of the location where you saved the .bin file
- Follow the instructions from Uploader Github Repo.

## Config Sparkfun with U-Center
1. Download U-Center
- Navigate to [U-Center Download Page](https://www.u-blox.com/en/product/u-center)
- Verify your u-blox chip number before download U-Center
2. 
