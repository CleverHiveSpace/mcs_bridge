# Automatic ROS2 Bag Recorder Service

Systemd service for automatically recording ROS2 bags on robot startup with disk space checking.


## Installation

### 1. Edit the recording script

Edit script rosbag_recorder.sh. Set up parameters and topics that should be recorded.
Record not compressed image only if needed and remember about camera_info.
"/tf" and "/tf_static" is recommended to log.

```bash
# Record rosbag
ros2 bag record -o "$DATE_NAME" -d $DURATION \
    /battery \
    /cmd_vel \
    /diagnostics \
    /oak/imu/data \
    /oak/rgb/camera_info \
    /oak/rgb/image_raw/compressed \
    /odometry/filtered \
    /tf \
    /tf_static &
```

### 2. Check all paths in rosbag-recorder.service
```bash
WorkingDirectory=/home/husarion/
ExecStart=/home/husarion/mcs_bridge/rosbag_recorder/rosbag_recorder.bash
```

### 3. Copy rosbag-recorder.service
```bash
# Copy service file to systemd
sudo cp rosbag-recorder.service /etc/systemd/system/rosbag-recorder.service
# Reload systemd daemon
sudo systemctl daemon-reload

# Enable service to start on boot
sudo systemctl enable rosbag-recorder.service

# Start the service immediately
sudo systemctl start rosbag-recorder.service

# Check service status
sudo systemctl status rosbag-recorder.service
```

## Usage

### Check service status
```bash
sudo systemctl status rosbag-recorder.service
```

### View logs in real-time
```bash
# View service logs
journalctl -u rosbag-recorder.service -f
```

### Stop recording
```bash
sudo systemctl stop rosbag-recorder.service
```

### Start recording manually
```bash
sudo systemctl start rosbag-recorder.service
```

### Restart the service
```bash
sudo systemctl restart rosbag-recorder.service
```

### Disable autostart on boot
```bash
sudo systemctl disable rosbag-recorder.service
```

### Enable autostart again
```bash
sudo systemctl enable rosbag-recorder.service
```