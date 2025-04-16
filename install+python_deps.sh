#!/bin/bash
echo "Installing Python dependencies for SparkFunRTKExpress..."

if ! command -v pip &> /dev/null; then
    echo "Error: pip is not installed. Please install pip first."
    exit 1
fi

echo "Installing pyserial..."
pip install pyserial

echo "Installing numpy..."
pip install numpy

echo "Installing matplotlib..."
pip install matplotlib

echo "Python dependencies installed successfully!"
echo "You can now proceed with building the SparkFunRTKExpress package."

exit 0