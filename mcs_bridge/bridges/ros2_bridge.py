"""ROS2 bridge implementation for telemetry data."""

import math
from re import A
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from .base import TelemetryBridge
from ..config import (
    ODOM_TOPIC,
    IMU_TOPIC,
    MAX_PUBLISH_RATE,
    POSITION_MULTIPLIER,
    POSITION_X_OFFSET,
    POSITION_Y_OFFSET,
    HEADING_OFFSET,
    IMU_ACCEL_X_NAME,
    IMU_ACCEL_Y_NAME,
    IMU_ACCEL_Z_NAME,
    IMU_ACCEL_UNIT,
)


def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw angle in radians."""
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


class ROS2TelemetryNode(Node):
    """ROS2 node that subscribes to telemetry topics and forwards to MCS."""

    def __init__(self, rover_id, client):
        super().__init__("mcs_bridge_node")
        self.rover_id = rover_id
        self.client = client

        # Rate limiting: track last publish time for each topic
        self.min_publish_interval = (
            1.0 / MAX_PUBLISH_RATE if MAX_PUBLISH_RATE > 0 else 0
        )
        self.last_odom_publish_time = 0.0
        self.last_imu_publish_time = 0.0

        # Pre-calculate heading offset in radians for coordinate rotation
        self.heading_offset_rad = math.radians(HEADING_OFFSET)
        self.cos_heading = math.cos(self.heading_offset_rad)
        self.sin_heading = math.sin(self.heading_offset_rad)

        self.odom_subscription = self.create_subscription(
            Odometry, ODOM_TOPIC, self.odometry_callback, 10
        )

        self.imu_subscription = self.create_subscription(
            Imu, IMU_TOPIC, self.imu_callback, 10
        )

        self.get_logger().info(f"MCS Bridge Node initialized for rover: {rover_id}")
        self.get_logger().info(f"Subscribed to {ODOM_TOPIC}")
        self.get_logger().info(f"Subscribed to {IMU_TOPIC}")
        self.get_logger().info(
            f"Max publish rate: {MAX_PUBLISH_RATE} Hz (min interval: {self.min_publish_interval:.2f}s)"
        )
        if (
            POSITION_MULTIPLIER != 1.0
            or POSITION_X_OFFSET != 0.0
            or POSITION_Y_OFFSET != 0.0
            or HEADING_OFFSET != 0.0
        ):
            self.get_logger().info(
                f"Position transform: multiplier={POSITION_MULTIPLIER}, "
                f"offset(longitude={POSITION_X_OFFSET}, latitude={POSITION_Y_OFFSET}), "
                f"heading_offset={HEADING_OFFSET}°"
            )

    def odometry_callback(self, msg):
        """Handle odometry messages."""
        try:
            # Rate limiting: check if enough time has passed since last publish
            current_time = time.time()
            if current_time - self.last_odom_publish_time < self.min_publish_interval:
                return  # Drop this message

            self.last_odom_publish_time = current_time

            # Get robot's local coordinates (in meters)
            local_x = msg.pose.pose.position.x
            local_y = msg.pose.pose.position.y

            # Rotate from robot's local frame to geographical frame (East, North)
            # heading_offset: angle from North to robot's X-axis
            # In ROS: X is forward, Y is left
            # When heading_offset=0°: X points North, Y points West
            # Note: Swapped East/North to fix 90° rotation issue

            # x increases northbound, decreases southbound
            # y increases westbound, decreases eastbound
            north_component = local_x * self.cos_heading + local_y * self.sin_heading
            west_component = local_x * self.sin_heading - local_y * self.cos_heading

            x = (north_component * POSITION_MULTIPLIER) + POSITION_X_OFFSET
            y = (west_component * POSITION_MULTIPLIER) + POSITION_Y_OFFSET
            z = msg.pose.pose.position.z

            orientation = msg.pose.pose.orientation
            yaw = quaternion_to_yaw(
                orientation.x, orientation.y, orientation.z, orientation.w
            )
            angle_deg = -math.degrees(yaw) % 360

            if self.client.send_position(self.rover_id, x, y, z, angle_deg):
                self.get_logger().info(
                    f"→ Position: x={x:.2f}, y={y:.2f}, z={z:.2f}, angle={angle_deg:.1f}°"
                )
            else:
                self.get_logger().warn(
                    "WebSocket not connected, cannot send position data"
                )
        except Exception as e:
            self.get_logger().error(f"Error in odometry callback: {e}")

    def imu_callback(self, msg):
        """Handle IMU messages."""
        try:
            # Rate limiting: check if enough time has passed since last publish
            current_time = time.time()
            if current_time - self.last_imu_publish_time < self.min_publish_interval:
                return  # Drop this message

            self.last_imu_publish_time = current_time

            accel_x = msg.linear_acceleration.x
            accel_y = msg.linear_acceleration.y
            accel_z = msg.linear_acceleration.z

            success = True
            success &= self.client.send_sensor(
                self.rover_id, IMU_ACCEL_X_NAME, accel_x, IMU_ACCEL_UNIT
            )
            success &= self.client.send_sensor(
                self.rover_id, IMU_ACCEL_Y_NAME, accel_y, IMU_ACCEL_UNIT
            )
            success &= self.client.send_sensor(
                self.rover_id, IMU_ACCEL_Z_NAME, accel_z, IMU_ACCEL_UNIT
            )

            if success:
                self.get_logger().info(
                    f"→ IMU: x={accel_x:.2f}, y={accel_y:.2f}, z={accel_z:.2f} m/s²"
                )
            else:
                self.get_logger().warn(
                    "WebSocket not connected, cannot send sensor data"
                )
        except Exception as e:
            self.get_logger().error(f"Error in IMU callback: {e}")


class ROS2Bridge(TelemetryBridge):
    """Bridge implementation using ROS2."""

    def __init__(self, rover_id, client):
        rclpy.init()
        self.node = ROS2TelemetryNode(rover_id, client)

    def spin(self):
        """Start processing ROS2 messages."""
        rclpy.spin(self.node)

    def cleanup(self):
        """Clean up ROS2 resources."""
        self.node.destroy_node()
        rclpy.shutdown()
