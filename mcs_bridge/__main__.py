#!/usr/bin/env python3
import argparse
import sys
from .config import USERNAME, PASSWORD, ROVER_ID


def parse_args():
    parser = argparse.ArgumentParser(
        description="MCS Bridge - Send telemetry data to MCS backend",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="\n"
        "Examples:\n"
        "  %(prog)s --bridge-type ros2 --username test1 --password test1 --rover-id r2d2\n"
        "  %(prog)s --bridge-type dummy -u admin -p secret123 -r rover01\n"
        "  %(prog)s --bridge-type dummy  # Uses credentials from .env file\n"
        "\n"
        "Bridge Types:\n"
        "  ros2  - Subscribe to ROS2 topics (/odometry/filtered, /imu_broadcaster/imu)\n"
        "  dummy - Generate random telemetry data (no ROS2 dependencies required)\n"
        "\n"
        "Credentials can be provided via:\n"
        "  - Command line arguments (--username, --password, --rover-id)\n"
        "  - Environment variables (USERNAME, PASSWORD, ROVER_ID)\n"
        "  - .env file in project root\n"
        "\n"
        "Press Ctrl+C to stop\n"
        "",
    )

    parser.add_argument(
        "--bridge-type",
        "-b",
        required=True,
        choices=["ros2", "dummy"],
        help="Bridge implementation to use (required)",
    )

    parser.add_argument(
        "--username",
        "-u",
        default=USERNAME,
        help="Username for authentication (default: from env/USERNAME)",
    )

    parser.add_argument(
        "--password",
        "-p",
        default=PASSWORD,
        help="Password for authentication (default: from env/PASSWORD)",
    )

    parser.add_argument(
        "--rover-id",
        "-r",
        default=ROVER_ID,
        help="Rover ID for telemetry data (default: from env/ROVER_ID)",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    # Validate that all required credentials are provided
    if not args.username:
        print("✗ Error: Username is required.")
        print(
            "   Provide via --username argument, USERNAME environment variable, or .env file"
        )
        sys.exit(1)

    if not args.password:
        print("✗ Error: Password is required.")
        print(
            "   Provide via --password argument, PASSWORD environment variable, or .env file"
        )
        sys.exit(1)

    if not args.rover_id:
        print("✗ Error: Rover ID is required.")
        print(
            "   Provide via --rover-id argument, ROVER_ID environment variable, or .env file"
        )
        sys.exit(1)

    print(f"Starting MCS Bridge for rover: {args.rover_id}")
    print(f"Bridge type: {args.bridge_type}")
    print(f"Username: {args.username}")
    print()

    # Import API client (always needed)
    from .api import MCSClient

    # Import bridge implementation based on selection
    if args.bridge_type == "ros2":
        try:
            from .bridges.ros2_bridge import ROS2Bridge as BridgeClass
        except ImportError as e:
            print("✗ Error: ROS2 bridge requires ROS2 dependencies.")
            print(f"   Missing dependency: {e}")
            print("\n   To use ROS2 bridge, you need to:")
            print("   1. Install ROS2 (https://docs.ros.org)")
            print("   2. Source your ROS2 installation")
            print("   3. Install Python ROS2 packages: rclpy, nav_msgs, sensor_msgs")
            print(
                "\n   Alternatively, use --bridge-type dummy for testing without ROS2."
            )
            sys.exit(1)
    elif args.bridge_type == "dummy":
        from .bridges.dummy_bridge import DummyBridge as BridgeClass
    else:
        print(f"✗ Unknown bridge type: {args.bridge_type}")
        sys.exit(1)

    # Connect to MCS backend
    client = MCSClient()

    print("Logging in...")
    if not client.login(args.username, args.password):
        print("✗ Failed to login. Exiting.")
        return

    print("Connecting to WebSocket...")
    if not client.connect():
        print("✗ Failed to connect to WebSocket. Exiting.")
        return

    print("✓ WebSocket connected successfully")
    print()

    # Create and start the bridge
    bridge = BridgeClass(args.rover_id, client)

    try:
        bridge.spin()
    except KeyboardInterrupt:
        print("\n\nStopping MCS Bridge...")
    finally:
        bridge.cleanup()
        client.disconnect()
        print("✓ Disconnected")


if __name__ == "__main__":
    main()
