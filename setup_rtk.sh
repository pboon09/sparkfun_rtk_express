#!/bin/bash

# Port fix script for SparkFun RTK Express Kit (u-blox ZED-F9P direct USB)

set -e  # Exit on error

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
SERIAL_DEVICE="/dev/ttyRTKExpress"
VERBOSE=false

# Function to print colored output
print_message() {
    local color=$1
    local message=$2
    echo -e "${color}${message}${NC}"
}

# Function to check if running as root
check_root() {
    if [ "$EUID" -ne 0 ]; then 
        print_message $RED "Error: Please run as root (use sudo)"
        exit 1
    fi
}

# Function to display usage
usage() {
    cat << EOF
Usage: $0 [OPTIONS]

SparkFun RTK Express Kit Port Fix Script (u-blox ZED-F9P Direct USB)
Creates a persistent /dev/ttyRTKExpress symlink for the RTK Express Kit

OPTIONS:
    -d, --device <path>       Serial device symlink path (default: /dev/ttyRTKExpress)
    -v, --verbose            Enable verbose output
    -h, --help               Display this help message

EXAMPLES:
    sudo $0                           # Setup RTK Express Kit port
    sudo $0 -d /dev/ttyGNSS          # Use custom symlink name
    sudo $0 -v                       # Verbose output

ABOUT YOUR RTK EXPRESS KIT:
    - u-blox ZED-F9P GNSS receiver with direct USB connection
    - Uses CDC ACM driver (appears as /dev/ttyACM*)
    - Vendor: u-blox AG (1546), Product: u-blox GNSS receiver (01a9)
    - Default NMEA output at 38400 baud
    - u-center configuration interface at 38400 baud

EOF
    exit 0
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -d|--device)
            SERIAL_DEVICE="$2"
            shift 2
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -h|--help)
            usage
            ;;
        *)
            print_message $RED "Unknown option: $1"
            usage
            ;;
    esac
done

# Get the actual username (not root when using sudo)
ACTUAL_USER="${SUDO_USER:-$USER}"

# Function to check if RTK Express Kit is connected
check_rtk_device() {
    print_message $BLUE "=== Checking for SparkFun RTK Express Kit ==="
    
    # Check specifically for u-blox ZED-F9P: 1546:01a9
    if lsusb | grep "1546:01a9"; then
        print_message $GREEN "✓ RTK Express Kit (u-blox ZED-F9P) found via USB"
        if $VERBOSE; then
            lsusb | grep "1546:01a9"
        fi
        return 0
    # Also check for any u-blox device
    elif lsusb | grep -i "u-blox"; then
        print_message $YELLOW "⚠ u-blox device found (checking if it's RTK Express Kit)"
        if $VERBOSE; then
            lsusb | grep -i "u-blox"
        fi
        return 0
    else
        print_message $YELLOW "⚠ No RTK Express Kit found via USB"
        print_message $YELLOW "Available USB devices:"
        lsusb
        print_message $YELLOW "Looking for: u-blox AG ZED-F9P GNSS receiver (1546:01a9)"
        return 1
    fi
}

# Function to create udev rules for RTK Express Kit
create_udev_rules() {
    print_message $BLUE "=== Creating udev Rules for RTK Express Kit ==="
    
    local UDEV_RULE_FILE="/etc/udev/rules.d/99-sparkfun-rtk-express.rules"
    local SYMLINK_NAME=$(basename "$SERIAL_DEVICE")
    
    # Check if rules already exist
    if [ -f "$UDEV_RULE_FILE" ]; then
        print_message $YELLOW "udev rules already exist, updating..."
    fi
    
    cat > "$UDEV_RULE_FILE" << EOF
# SparkFun RTK Express Kit USB device rules
# RTK Express Kit with u-blox ZED-F9P direct USB connection

# Primary rule: u-blox ZED-F9P GNSS receiver used in RTK Express Kit
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="$SYMLINK_NAME", MODE="0666", GROUP="dialout", TAG+="uaccess"

# Alternative rule using manufacturer string
SUBSYSTEM=="tty", ATTRS{manufacturer}=="u-blox AG - www.u-blox.com", ATTRS{product}=="u-blox GNSS receiver", SYMLINK+="$SYMLINK_NAME", MODE="0666", GROUP="dialout", TAG+="uaccess"

# Product string matching for u-blox GNSS receiver
SUBSYSTEM=="tty", ATTRS{product}=="u-blox GNSS receiver", SYMLINK+="$SYMLINK_NAME", MODE="0666", GROUP="dialout", TAG+="uaccess"

# Generic u-blox rule for RTK Express Kit (CDC ACM driver)
SUBSYSTEM=="tty", KERNEL=="ttyACM[0-9]*", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE="0666", GROUP="dialout", TAG+="uaccess"

# Fallback rule for any u-blox device (less specific)
SUBSYSTEM=="tty", KERNEL=="ttyACM[0-9]*", ATTRS{idVendor}=="1546", MODE="0666", GROUP="dialout", TAG+="uaccess"
EOF

    print_message $GREEN "✓ Created udev rules for RTK Express Kit (u-blox ZED-F9P)"
    
    # Add user to dialout group
    if ! groups "$ACTUAL_USER" | grep -q "dialout"; then
        print_message $YELLOW "Adding $ACTUAL_USER to dialout group..."
        usermod -a -G dialout "$ACTUAL_USER"
        print_message $GREEN "✓ Added $ACTUAL_USER to dialout group"
        print_message $YELLOW "Note: User needs to log out and back in for group change to take effect"
    else
        print_message $GREEN "✓ User $ACTUAL_USER already in dialout group"
    fi
    
    # Reload udev rules
    print_message $YELLOW "Reloading udev rules..."
    udevadm control --reload-rules
    udevadm trigger
    print_message $GREEN "✓ Reloaded udev rules"
}

# Function to find RTK Express Kit device
find_rtk_device() {
    local SYMLINK_NAME=$(basename "$SERIAL_DEVICE")
    
    # First try the symlink
    if [ -L "$SERIAL_DEVICE" ]; then
        print_message $GREEN "✓ Found RTK Express Kit at $SERIAL_DEVICE"
        return 0
    fi
    
    # Look for u-blox ZED-F9P devices (1546:01a9)
    for device in /dev/ttyACM*; do
        if [ -e "$device" ]; then
            # Check if it's the u-blox ZED-F9P by vendor/product ID
            local vendorid=$(udevadm info -q property -n "$device" 2>/dev/null | grep ID_VENDOR_ID | cut -d= -f2)
            local productid=$(udevadm info -q property -n "$device" 2>/dev/null | grep ID_MODEL_ID | cut -d= -f2)
            local manufacturer=$(udevadm info -q property -n "$device" 2>/dev/null | grep ID_VENDOR_FROM_DATABASE | cut -d= -f2)
            
            # Check for u-blox ZED-F9P (1546:01a9)
            if [[ "$vendorid" == "1546" ]] && [[ "$productid" == "01a9" ]]; then
                print_message $GREEN "✓ Found RTK Express Kit (u-blox ZED-F9P) at $device"
                return 0
            # Check for any u-blox device as fallback
            elif [[ "$vendorid" == "1546" ]]; then
                print_message $YELLOW "⚠ Found u-blox device at $device (Product ID: $productid)"
                return 0
            fi
        fi
    done
    
    print_message $RED "✗ No RTK Express Kit device found"
    print_message $YELLOW "Expected: u-blox ZED-F9P GNSS receiver (1546:01a9) on /dev/ttyACM*"
    return 1
}

# Function to verify setup
verify_setup() {
    print_message $BLUE "=== Verifying RTK Express Kit Port Setup ==="
    
    local SYMLINK_NAME=$(basename "$SERIAL_DEVICE")
    
    # Show available serial devices
    print_message $YELLOW "\nAvailable ACM serial devices:"
    ls -la /dev/ttyACM* 2>/dev/null || print_message $YELLOW "No ACM serial devices found"
    
    # Show our symlink if it exists
    if [ -L "$SERIAL_DEVICE" ]; then
        print_message $GREEN "\n✓ RTK Express Kit symlink created:"
        ls -la "$SERIAL_DEVICE"
    fi
    
    # Show device info if verbose
    if $VERBOSE && [ -L "$SERIAL_DEVICE" ]; then
        local real_device=$(readlink "$SERIAL_DEVICE")
        print_message $YELLOW "\nDevice information:"
        udevadm info -q property -n "$real_device" 2>/dev/null | grep -E "(ID_VENDOR|ID_MODEL|ID_SERIAL|ID_USB_DRIVER)" || true
    fi
    
    return 0
}

# Function to show usage examples specific to RTK Express Kit
show_usage_examples() {
    print_message $BLUE "\n=== RTK Express Kit Usage Examples ==="
    
    print_message $YELLOW "Device location:"
    if [ -L "$SERIAL_DEVICE" ]; then
        echo "   $SERIAL_DEVICE -> $(readlink $SERIAL_DEVICE)"
    else
        echo "   Check /dev/ttyACM* for your RTK Express Kit (u-blox ZED-F9P)"
    fi
    
    print_message $YELLOW "\nZED-F9P GNSS receiver connection:"
    echo "   sudo screen $SERIAL_DEVICE 38400"
    echo "   # or"
    echo "   sudo minicom -D $SERIAL_DEVICE -b 38400"
    
    print_message $YELLOW "\nAlternative baud rates to try:"
    echo "   38400  - Default NMEA output and u-center configuration"
    echo "   9600   - Some NMEA configurations"
    echo "   115200 - High-speed NMEA output"
    
    print_message $YELLOW "\nExpected behavior:"
    echo "   - NMEA sentences (\\$GNGGA, \\$GNRMC, etc.) at 38400 baud"
    echo "   - u-center software can connect for configuration"
    echo "   - Raw GNSS data and RTK corrections"
    
    print_message $YELLOW "\nRTK Express Kit specific notes:"
    echo "   - Direct connection to u-blox ZED-F9P GNSS receiver"
    echo "   - Supports RTK corrections for cm-level accuracy"
    echo "   - Multi-band L1/L2 GNSS reception"
    echo "   - Compatible with u-center for advanced configuration"
    echo "   - CDC ACM driver (no additional drivers needed on Linux)"
    
    print_message $YELLOW "\nTroubleshooting:"
    echo "   - Device should appear automatically with CDC ACM driver"
    echo "   - Verify device appears in 'lsusb' as 1546:01a9"
    echo "   - Try different USB cable if not detected"
    echo "   - Check dmesg for 'cdc_acm' driver messages"
    echo "   - Power cycle device if connection issues occur"
}

# Main execution
main() {
    print_message $GREEN "=== SparkFun RTK Express Kit Port Fix Script ==="
    print_message $YELLOW "Target device symlink: $SERIAL_DEVICE"
    print_message $YELLOW "Hardware: u-blox ZED-F9P GNSS receiver (Direct USB)"
    print_message $YELLOW "Expected: 1546:01a9 on /dev/ttyACM*"
    echo
    
    # Check root privileges
    check_root
    
    # Check for RTK Express Kit device
    if ! check_rtk_device; then
        print_message $YELLOW "Please connect your RTK Express Kit and try again"
        print_message $YELLOW "Device should appear as u-blox GNSS receiver"
        print_message $YELLOW "Expected USB ID: 1546:01a9 (u-blox AG)"
    fi
       
    # Create udev rules
    create_udev_rules
    
    # Wait for udev to process
    sleep 2
    
    # Find RTK Express Kit device
    if ! find_rtk_device; then
        print_message $YELLOW "Please unplug and replug your RTK Express Kit, then run this script again"
        print_message $YELLOW "The device should appear as /dev/ttyACM* with vendor 1546:01a9"
    fi

    # Verify setup
    if verify_setup; then
        print_message $GREEN "\n✓ RTK Express Kit port setup completed!"
        
        # Show usage examples
        show_usage_examples
        
        print_message $YELLOW "\nNext steps:"
        print_message $YELLOW "1. Connect to ZED-F9P at 38400 baud to see NMEA data"
        print_message $YELLOW "2. Use u-center software for advanced configuration"
        print_message $YELLOW "3. Connect L1/L2 antenna for optimal GNSS reception"
        print_message $YELLOW "4. Configure RTK corrections for cm-level accuracy"
        
    else
        print_message $RED "\n✗ RTK Express Kit port setup completed with warnings"
        print_message $YELLOW "Check 'dmesg | tail' for USB connection issues"
        print_message $YELLOW "Verify device appears with CDC ACM driver"
    fi
}

# Run main function
main