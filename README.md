# MCS Bridge

Bridges telemetry data (position, IMU, and sensors) to the MCS backend via WebSocket.

## Run with Docker

1. Build:
   ```bash
   docker build -t mcs-bridge .
   ```
2. Verify:
   ```bash
   docker run -it mcs-bridge --help
   ```
3. Run with dummy bridge:
   ```bash
   docker run -it mcs-bridge \
    --bridge-type dummy \
    --username username \
    --password password \
    --rover-id rover_id
   ```
   OR
   ```
   docker run -it \
    -e USERNAME=username \
    -e PASSWORD=password \
    -e ROVER_ID=rover_id \
    mcs-bridge \
      --bridge-type dummy
   ```

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

## Usage

```bash
python3 mcs_bridge -b dummy
```
