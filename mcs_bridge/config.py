import os
from pathlib import Path
from dotenv import load_dotenv


env_path = Path(__file__).parent.parent / ".env"
load_dotenv(env_path, override=True)


BACKEND_URL = os.getenv("BACKEND_URL", "https://api.rovers.website")
WS_NAMESPACE = os.getenv("WS_NAMESPACE", "/ws/robot")

# Authentication credentials
USERNAME = os.getenv("USERNAME")
PASSWORD = os.getenv("PASSWORD")
ROVER_ID = os.getenv("ROVER_ID")

# ROS2 topic configuration
ODOM_TOPIC = os.getenv("ODOM_TOPIC", "/odometry/filtered")
IMU_TOPIC = os.getenv("IMU_TOPIC", "/imu_broadcaster/imu")

# Rate limiting configuration (Hz)
MAX_PUBLISH_RATE = float(os.getenv("MAX_PUBLISH_RATE", "4"))

# Position transformation configuration
# ROS odometry is in meters, but offset coordinates are in geographical units
# POSITION_MULTIPLIER: Converts ROS meters to geographical coordinate units
#   - For lat/lon (degrees): ~0.000009 (1m ≈ 0.000009 degrees)
#   - For UTM (meters): 1.0 (1m = 1m)
#   - Adjust based on your coordinate system
POSITION_MULTIPLIER = float(os.getenv("POSITION_MULTIPLIER", "0.000009"))

# Starting coordinates in geographical coordinate system
# X_OFFSET = Longitude (east-west, typically -180 to +180)
# Y_OFFSET = Latitude (north-south, typically -90 to +90)
POSITION_X_OFFSET = float(os.getenv("POSITION_X_OFFSET", "53.175"))  # Longitude
POSITION_Y_OFFSET = float(os.getenv("POSITION_Y_OFFSET", "16.725833"))  # Latitude

# Initial heading/orientation of robot's local coordinate system
# HEADING_OFFSET: Angle (in degrees) from North to robot's X-axis
#   - 0°: Robot X-axis points North, Y-axis points East
#   - 90°: Robot X-axis points East, Y-axis points South
#   - 180°: Robot X-axis points South, Y-axis points West
#   - 270°: Robot X-axis points West, Y-axis points North
# This rotates the robot's local coordinates to align with geographical coordinates
HEADING_OFFSET = float(os.getenv("HEADING_OFFSET", "0.0"))


# Sensor configuration
IMU_ACCEL_X_NAME = os.getenv("IMU_ACCEL_X_NAME", "imu_accel_x")
IMU_ACCEL_Y_NAME = os.getenv("IMU_ACCEL_Y_NAME", "imu_accel_y")
IMU_ACCEL_Z_NAME = os.getenv("IMU_ACCEL_Z_NAME", "imu_accel_z")
IMU_ACCEL_UNIT = os.getenv("IMU_ACCEL_UNIT", "m/s2")
