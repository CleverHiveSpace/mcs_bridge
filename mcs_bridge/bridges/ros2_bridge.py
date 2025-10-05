"""ROS2 bridge implementation for telemetry data."""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from .base import TelemetryBridge
from ..config import (
    ODOM_TOPIC,
    IMU_TOPIC,
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

        self.odom_subscription = self.create_subscription(
            Odometry, ODOM_TOPIC, self.odometry_callback, 10
        )

        self.imu_subscription = self.create_subscription(
            Imu, IMU_TOPIC, self.imu_callback, 10
        )

        self.get_logger().info(f"MCS Bridge Node initialized for rover: {rover_id}")
        self.get_logger().info(f"Subscribed to {ODOM_TOPIC}")
        self.get_logger().info(f"Subscribed to {IMU_TOPIC}")

    def odometry_callback(self, msg):
        """Handle odometry messages."""
        try:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z

            orientation = msg.pose.pose.orientation
            yaw = quaternion_to_yaw(
                orientation.x, orientation.y, orientation.z, orientation.w
            )
            angle_deg = math.degrees(yaw) % 360

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
