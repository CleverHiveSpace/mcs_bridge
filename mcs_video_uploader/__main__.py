import argparse
import logging
import queue
import sys

import rclpy

from . import __version__
from .api import VideoClient
from .config import (
    BACKEND_URL,
    PASSWORD,
    USERNAME,
    VIDEO_ACK_TIMEOUT,
    VIDEO_CAMERA_ID,
    VIDEO_IDLE_FLUSH_SECONDS,
    VIDEO_MAX_QUEUE,
    VIDEO_SEGMENT_SECONDS,
    VIDEO_TOPIC,
)
from .processor import SegmentProcessor
from .ros2_node import VideoSegmenterNode


def _configure_logging(verbose: bool):
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="[%(asctime)s] %(levelname)s %(name)s: %(message)s",
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="ROS2 video uploader streaming segments to the backend",
    )
    parser.add_argument("--username", "-u", default=USERNAME, help="Authentication username")
    parser.add_argument("--password", "-p", default=PASSWORD, help="Authentication password")
    parser.add_argument("--camera-id", "-c", default=VIDEO_CAMERA_ID, help="Camera identifier")
    parser.add_argument("--topic", "-t", default=VIDEO_TOPIC, help="ROS2 topic with sensor_msgs/Image data")
    parser.add_argument(
        "--segment-seconds",
        type=float,
        default=VIDEO_SEGMENT_SECONDS,
        help="Length of each video segment in seconds",
    )
    parser.add_argument(
        "--idle-flush-seconds",
        type=float,
        default=VIDEO_IDLE_FLUSH_SECONDS,
        help="Flush partial segments after this idle time",
    )
    parser.add_argument(
        "--queue-size",
        type=int,
        default=VIDEO_MAX_QUEUE,
        help="Maximum number of segments waiting to be processed",
    )
    parser.add_argument("--verbose", "-v", action="store_true", help="Enable debug logging")
    parser.add_argument("--version", action="store_true", help="Print version and exit")
    return parser.parse_args()


def main():
    args = parse_args()

    if args.version:
        print(f"mcs-video-uploader {__version__}")
        return

    _configure_logging(args.verbose)
    logger = logging.getLogger("mcs_video_uploader")

    if not args.username:
        logger.error("Username is required (provide via --username or env USERNAME)")
        sys.exit(1)

    if not args.password:
        logger.error("Password is required (provide via --password or env PASSWORD)")
        sys.exit(1)

    logger.info("Starting video uploader (backend: %s)", BACKEND_URL)

    client = VideoClient()

    logger.info("Logging in as %s", args.username)
    if not client.login(args.username, args.password):
        logger.error("Failed to authenticate, terminating")
        sys.exit(1)

    logger.info("Connecting to WebSocket namespace")
    if not client.connect():
        logger.error("Failed to connect to WebSocket, terminating")
        sys.exit(1)

    segment_queue: queue.Queue = queue.Queue(maxsize=args.queue_size)
    processor = SegmentProcessor(
        client=client,
        camera_id=args.camera_id,
        segment_queue=segment_queue,
        ack_timeout=VIDEO_ACK_TIMEOUT,
    )
    processor.start()

    rclpy.init()
    node = VideoSegmenterNode(
        topic=args.topic,
        camera_id=args.camera_id,
        segment_queue=segment_queue,
        segment_seconds=args.segment_seconds,
        idle_flush_seconds=args.idle_flush_seconds,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Interrupt received, shutting down")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

        processor.stop()
        try:
            segment_queue.join()
        except KeyboardInterrupt:
            logger.warning("Interrupted while draining segment queue")
        processor.join(timeout=5)
        if processor.is_alive():
            logger.warning("Segment processor thread is still running")

        client.disconnect()
        logger.info("Video uploader stopped cleanly")


if __name__ == "__main__":
    main()
