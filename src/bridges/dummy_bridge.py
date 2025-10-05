"""Dummy bridge implementation for testing without ROS2 dependencies."""

import time
import math
import random

from .base import TelemetryBridge


class DummyBridge(TelemetryBridge):
    """Bridge implementation that generates and sends dummy telemetry data."""

    def __init__(self, rover_id, client):
        self.rover_id = rover_id
        self.client = client
        self.running = False

        # Initial position
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.angle = 0.0

        # Movement parameters
        self.velocity = 0.5  # m/s
        self.angular_velocity = 10.0  # degrees/s

        print(f"Dummy Bridge initialized for rover: {rover_id}")
        print("This bridge will generate random telemetry data for testing.")

    def spin(self):
        """Start generating and sending dummy data."""
        self.running = True
        print("\nDummy Bridge started. Generating telemetry data...")
        print("Press Ctrl+C to stop\n")

        iteration = 0
        try:
            while self.running:
                # Update position (random walk)
                direction = math.radians(self.angle)
                self.x += self.velocity * math.cos(direction)
                self.y += self.velocity * math.sin(direction)
                self.z += random.uniform(-0.1, 0.1)  # Small random z changes

                # Update angle (random turn)
                self.angle += random.uniform(
                    -self.angular_velocity, self.angular_velocity
                )
                self.angle = self.angle % 360

                # Send position
                if self.client.send_position(
                    self.rover_id, self.x, self.y, self.z, self.angle
                ):
                    print(
                        f"→ Position: x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}, angle={self.angle:.1f}°"
                    )
                else:
                    print("✗ Failed to send position data")

                # Generate random IMU data
                accel_x = random.uniform(-2.0, 2.0)
                accel_y = random.uniform(-2.0, 2.0)
                accel_z = 9.81 + random.uniform(-0.5, 0.5)  # Gravity + noise

                # Send IMU data
                success = True
                success &= self.client.send_sensor(
                    self.rover_id, "imu_accel_x", accel_x, "m/s²"
                )
                success &= self.client.send_sensor(
                    self.rover_id, "imu_accel_y", accel_y, "m/s²"
                )
                success &= self.client.send_sensor(
                    self.rover_id, "imu_accel_z", accel_z, "m/s²"
                )

                if success:
                    print(
                        f"→ IMU: x={accel_x:.2f}, y={accel_y:.2f}, z={accel_z:.2f} m/s²"
                    )
                else:
                    print("✗ Failed to send sensor data")

                # Send additional dummy sensor
                if iteration % 5 == 0:
                    battery_level = max(0, 100 - (iteration / 10))
                    self.client.send_sensor(
                        self.rover_id, "battery_level", battery_level, "%"
                    )
                    print(f"→ Battery: {battery_level:.1f}%")

                print()
                iteration += 1
                time.sleep(1)  # Send data every second

        except KeyboardInterrupt:
            print("\n\nStopping Dummy Bridge...")
            self.running = False

    def cleanup(self):
        """Clean up resources."""
        self.running = False
        print("Dummy Bridge cleaned up")
