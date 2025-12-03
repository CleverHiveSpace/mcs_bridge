#!/bin/bash

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Minimum required space in GB
MIN_SPACE_GB=2

# Recording duration in seconds (300 = 5 minutes)
DURATION=300

# Cleanup function
cleanup() {
    echo ""
    echo "Recording interrupted! Attempting to finalize rosbag..."
    if [ ! -z "$BAG_PID" ]; then
        kill -SIGINT $BAG_PID
        wait $BAG_PID
    fi
    echo "Rosbag finalized. Exiting."
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "[$(date)] Starting rosbag recording service..."

# Check disk space
AVAILABLE_SPACE=$(df -BG / | awk 'NR==2 {print $4}' | sed 's/G//')

echo "Available disk space: ${AVAILABLE_SPACE}GB"
echo "Required disk space: ${MIN_SPACE_GB}GB"

if [ "$AVAILABLE_SPACE" -lt "$MIN_SPACE_GB" ]; then
    echo "ERROR: Not enough disk space available!"
    exit 1
fi

mkdir rosbag
cd rosbag
# Create rosbag name with date: rosbag_2024-11-24_14-30-15
DATE_NAME=$(date +"%Y-%m-%d_%H-%M-%S")

echo "Starting recording: $DATE_NAME"

# Record rosbag
ros2 bag record -o "$DATE_NAME" -d $DURATION \
    /firmware/battery_averaged \
    /imu/data \
    /tf \
    /tf_static \
    /camera/image_rect_color/compressed \
    /camera/camera_info \
    /wheel_odom_with_covariance \
    /cmd_vel &

BAG_PID=$!
wait $BAG_PID

echo "[$(date)] Recording completed: $DATE_NAME"