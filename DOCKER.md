# Docker Setup for MCS Bridge

This document explains how to build and run the MCS Bridge using Docker with ROS2 Humble support.

## Quick Start

### 1. Build the Docker Image

```bash
docker build -t mcs-bridge .
```

### 2. Run with Docker Compose (Recommended)

Create a `.env` file with your credentials:

```bash
# .env file
USERNAME=your_username
PASSWORD=your_password
ROVER_ID=your_rover_id
BACKEND_URL=https://api.rovers.website
WS_NAMESPACE=/ws/robot
```

Then run:

```bash
# For dummy bridge (testing)
docker-compose --profile dummy up

# For ROS2 bridge (production)
docker-compose --profile ros2 up

# For default service (shows help)
docker-compose up
```

### 3. Run with Docker directly

```bash
# Dummy bridge with command line arguments
docker run --rm --network host \
  -e USERNAME=test_user \
  -e PASSWORD=test_pass \
  -e ROVER_ID=test_rover \
  mcs-bridge --bridge-type dummy

# ROS2 bridge with command line arguments
docker run --rm --network host \
  -e USERNAME=your_user \
  -e PASSWORD=your_pass \
  -e ROVER_ID=your_rover \
  mcs-bridge --bridge-type ros2
```

## Configuration

### Environment Variables

| Variable           | Default                      | Description                        |
| ------------------ | ---------------------------- | ---------------------------------- |
| `USERNAME`         | -                            | Authentication username (required) |
| `PASSWORD`         | -                            | Authentication password (required) |
| `ROVER_ID`         | -                            | Rover identifier (required)        |
| `BACKEND_URL`      | `https://api.rovers.website` | MCS backend URL                    |
| `WS_NAMESPACE`     | `/ws/robot`                  | WebSocket namespace                |
| `ODOM_TOPIC`       | `/odometry/filtered`         | ROS2 odometry topic                |
| `IMU_TOPIC`        | `/imu_broadcaster/imu`       | ROS2 IMU topic                     |
| `IMU_ACCEL_X_NAME` | `imu_accel_x`                | IMU X acceleration sensor name     |
| `IMU_ACCEL_Y_NAME` | `imu_accel_y`                | IMU Y acceleration sensor name     |
| `IMU_ACCEL_Z_NAME` | `imu_accel_z`                | IMU Z acceleration sensor name     |
| `IMU_ACCEL_UNIT`   | `m/s2`                       | IMU acceleration unit              |

### Command Line Arguments

The Docker container accepts all the same command line arguments as the Python script:

```bash
# Show help
docker run --rm mcs-bridge --help

# Dummy bridge with specific credentials
docker run --rm --network host \
  mcs-bridge --bridge-type dummy \
  --username test_user \
  --password test_pass \
  --rover-id test_rover

# ROS2 bridge with custom topics
docker run --rm --network host \
  -e ODOM_TOPIC=/custom/odom \
  -e IMU_TOPIC=/custom/imu \
  mcs-bridge --bridge-type ros2 \
  --username prod_user \
  --password prod_pass \
  --rover-id rover_001
```

## Docker Compose Services

The `docker-compose.yml` file provides three services:

### 1. `mcs-bridge` (Default)

- Shows help by default
- Can be configured with environment variables
- Use: `docker-compose up`

### 2. `mcs-bridge-ros2` (ROS2 Profile)

- Pre-configured for ROS2 bridge
- Use: `docker-compose --profile ros2 up`

### 3. `mcs-bridge-dummy` (Dummy Profile)

- Pre-configured for dummy bridge with test credentials
- Use: `docker-compose --profile dummy up`

## Network Configuration

The container uses `network_mode: host` to:

- Access ROS2 topics from the host system
- Connect to the MCS backend
- Allow ROS2 nodes to communicate properly

## Volume Mounts

### Environment File

```yaml
volumes:
  - ./.env:/workspace/.env:ro
```

### Custom Configuration

```yaml
volumes:
  - ./config:/workspace/config:ro
```

## Examples

### Development/Testing

```bash
# Quick test with dummy data
docker-compose --profile dummy up

# Test with custom environment
USERNAME=dev_user PASSWORD=dev_pass ROVER_ID=dev_rover docker-compose --profile dummy up
```

### Production Deployment

```bash
# Create production .env file
cat > .env << EOF
USERNAME=production_user
PASSWORD=secure_password
ROVER_ID=rover_001
BACKEND_URL=https://api.rovers.website
ODOM_TOPIC=/odometry/filtered
IMU_TOPIC=/imu_broadcaster/imu
EOF

# Deploy ROS2 bridge
docker-compose --profile ros2 up -d
```

### Custom ROS2 Topics

```bash
# Override ROS2 topics
docker run --rm --network host \
  -e USERNAME=user -e PASSWORD=pass -e ROVER_ID=rover \
  -e ODOM_TOPIC=/robot/odom \
  -e IMU_TOPIC=/robot/imu \
  mcs-bridge --bridge-type ros2
```

### Health Monitoring

The container includes a health check:

```bash
# Check container health
docker ps
docker inspect mcs-bridge --format='{{.State.Health.Status}}'
```

## Troubleshooting

### ROS2 Connection Issues

1. Ensure ROS2 is running on the host:

   ```bash
   ros2 topic list
   ```

2. Check if topics are available:
   ```bash
   ros2 topic echo /odometry/filtered
   ros2 topic echo /imu_broadcaster/imu
   ```

### Authentication Issues

1. Verify credentials in `.env` file
2. Test backend connectivity:
   ```bash
   curl -X POST https://api.rovers.website/api/auth/login \
     -H "Content-Type: application/json" \
     -d '{"username":"your_user","password":"your_pass"}'
   ```

### Container Logs

```bash
# View logs
docker-compose logs -f

# View logs for specific service
docker-compose logs -f mcs-bridge-ros2
```

## Building for Different Architectures

```bash
# Build for ARM64 (e.g., Raspberry Pi)
docker buildx build --platform linux/arm64 -t mcs-bridge:arm64 .

# Build for AMD64
docker buildx build --platform linux/amd64 -t mcs-bridge:amd64 .
```
