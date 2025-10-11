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

POSITION_MULTIPLIER = float(os.getenv("POSITION_MULTIPLIER", "0.09"))
POSITION_X_OFFSET = float(os.getenv("POSITION_X_OFFSET", "53.383328"))
POSITION_Y_OFFSET = float(os.getenv("POSITION_Y_OFFSET", "-16.951904"))

# Sensor configuration
IMU_ACCEL_X_NAME = os.getenv("IMU_ACCEL_X_NAME", "imu_accel_x")
IMU_ACCEL_Y_NAME = os.getenv("IMU_ACCEL_Y_NAME", "imu_accel_y")
IMU_ACCEL_Z_NAME = os.getenv("IMU_ACCEL_Z_NAME", "imu_accel_z")
IMU_ACCEL_UNIT = os.getenv("IMU_ACCEL_UNIT", "m/s2")
