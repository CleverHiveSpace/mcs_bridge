import os
from pathlib import Path
from dotenv import load_dotenv


env_path = Path(__file__).parent.parent / ".env"
load_dotenv(env_path)


BACKEND_URL = os.getenv("BACKEND_URL", "https://api.rovers.website")
WS_NAMESPACE = os.getenv("WS_NAMESPACE", "/ws/robot")

# Authentication credentials
USERNAME = os.getenv("USERNAME")
PASSWORD = os.getenv("PASSWORD")
ROVER_ID = os.getenv("ROVER_ID")

# ROS2 topic configuration
ODOM_TOPIC = os.getenv("ODOM_TOPIC", "/odometry/filtered")
IMU_TOPIC = os.getenv("IMU_TOPIC", "/imu_broadcaster/imu")

# Sensor configuration
IMU_ACCEL_X_NAME = os.getenv("IMU_ACCEL_X_NAME", "imu_accel_x")
IMU_ACCEL_Y_NAME = os.getenv("IMU_ACCEL_Y_NAME", "imu_accel_y")
IMU_ACCEL_Z_NAME = os.getenv("IMU_ACCEL_Z_NAME", "imu_accel_z")
IMU_ACCEL_UNIT = os.getenv("IMU_ACCEL_UNIT", "m/s2")
