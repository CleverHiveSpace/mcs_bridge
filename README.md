# MCS Bridge

Bridges telemetry data (position, IMU, and sensors) to the MCS backend via WebSocket.

## Architecture

- `main.py` - Entry point with CLI argument parsing and bridge selection
- `api.py` - MCS backend communication (HTTP/WebSocket)
- `config.py` - Configuration loader from .env
- `bridges/` - Bridge implementations
  - `base.py` - Abstract base class for bridges
  - `ros2_bridge.py` - ROS2 implementation (subscribes to odometry and IMU topics)
  - `dummy_bridge.py` - Dummy implementation (generates random data for testing)

## Installation

### Core Dependencies

Install the core dependencies (required for all bridge types):

```bash
pip install -r requirements.txt
```

### ROS2 Dependencies (Optional)

Only required if using `--bridge-type ros2`:

1. Install ROS2: https://docs.ros.org/en/humble/Installation.html
2. Source your ROS2 installation: `source /opt/ros/humble/setup.bash`
3. ROS2 Python packages (rclpy, nav_msgs, sensor_msgs) should be available through ROS2

## Usage

### With Dummy Bridge (No ROS2 Required)

Perfect for testing without ROS2 dependencies. Generates random telemetry data:

```bash
# Using command line arguments
python3 src/main.py --bridge-type dummy --username test1 --password test1 --rover-id r2d2

# Using environment variables or .env file
python3 src/main.py --bridge-type dummy
```

### With ROS2 Bridge

Subscribes to ROS2 topics and forwards real telemetry data:

```bash
# Using command line arguments
python3 src/main.py --bridge-type ros2 --username test1 --password test1 --rover-id r2d2

# Using environment variables or .env file
python3 src/main.py --bridge-type ros2
```

The ROS2 bridge subscribes to:

- `/odometry/filtered` - for position data (x, y, z, angle)
- `/imu_broadcaster/imu` - for IMU acceleration data

### Short Options

```bash
python3 src/main.py -b dummy -u admin -p secret123 -r rover01
python3 src/main.py -b ros2 -u admin -p secret123 -r rover01
```

## Configuration

### Environment Variables

Credentials and configuration can be provided via:

1. **Command line arguments** (highest priority)
2. **Environment variables**
3. **`.env` file** in project root (lowest priority)

Create a `.env` file to customize settings:

```bash
# Backend Configuration
BACKEND_URL=https://api.rovers.website
WS_NAMESPACE=/ws/robot

# Authentication (optional - can be provided via command line)
USERNAME=your_username
PASSWORD=your_password
ROVER_ID=your_rover_id

# ROS2 Topic Configuration
ODOM_TOPIC=/odometry/filtered
IMU_TOPIC=/imu_broadcaster/imu

# Sensor Configuration
IMU_ACCEL_X_NAME=imu_accel_x
IMU_ACCEL_Y_NAME=imu_accel_y
IMU_ACCEL_Z_NAME=imu_accel_z
IMU_ACCEL_UNIT=m/s2
```

### Credential Sources

You can provide credentials in any of these ways:

**Command line (recommended for security):**

```bash
python3 src/main.py -b dummy -u myuser -p mypass -r rover01
```

**Environment variables:**

```bash
export USERNAME=myuser
export PASSWORD=mypass
export ROVER_ID=rover01
python3 src/main.py -b dummy
```

**`.env` file:**

```bash
# Create .env file with credentials
echo "USERNAME=myuser" > .env
echo "PASSWORD=mypass" >> .env
echo "ROVER_ID=rover01" >> .env
python3 src/main.py -b dummy
```

## Bridge Types

### dummy

- No external dependencies (beyond core Python packages)
- Generates random position and IMU data
- Useful for testing MCS backend connectivity
- Includes random walk simulation for position
- Sends battery level sensor data

### ros2

- Requires ROS2 installation
- Subscribes to real ROS2 topics
- Forwards odometry and IMU data to MCS backend
- Production-ready for actual rover deployments
