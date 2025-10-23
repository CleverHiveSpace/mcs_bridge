import logging
import queue
import time
from typing import Optional

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image

from .segments import Segment

logger = logging.getLogger(__name__)


class SegmentAccumulator:
    """Collect frames until they form a complete segment."""

    def __init__(self, segment_length: float):
        self._segment_length = segment_length
        self._frames = []
        self._timestamps = []

    def add_frame(self, frame, timestamp: float) -> Optional[Segment]:
        self._frames.append(frame)
        self._timestamps.append(timestamp)

        if self._is_full():
            return self.flush()
        return None

    def flush(self) -> Optional[Segment]:
        if not self._frames:
            return None

        segment = Segment(frames=list(self._frames), timestamps=list(self._timestamps))
        self._frames.clear()
        self._timestamps.clear()
        return segment

    def _is_full(self) -> bool:
        if len(self._timestamps) < 2:
            return False
        return (self._timestamps[-1] - self._timestamps[0]) >= self._segment_length

    def has_frames(self) -> bool:
        return bool(self._frames)


class VideoSegmenterNode(Node):
    """ROS2 node that converts Image messages into video segments."""

    def __init__(
        self,
        *,
        topic: str,
        camera_id: str,
        segment_queue: queue.Queue,
        segment_seconds: float,
        idle_flush_seconds: float,
    ):
        super().__init__("mcs_video_segmenter")
        self._camera_id = camera_id
        self._segment_queue = segment_queue
        self._segment_seconds = segment_seconds
        self._idle_flush_seconds = idle_flush_seconds
        self._bridge = CvBridge()
        self._accumulator = SegmentAccumulator(segment_seconds)
        self._last_frame_time = 0.0

        self.create_subscription(Image, topic, self._image_callback, 10)
        self.create_timer(self._idle_flush_seconds, self._idle_timer_callback)

        self.get_logger().info(
            f"Video segmenter configured for topic {topic} "
            f"(segment={segment_seconds:.1f}s, idle flush={idle_flush_seconds:.1f}s)"
        )

    def _image_callback(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error("Failed to convert image: %s", exc)
            return

        frame_timestamp = time.time()
        self._last_frame_time = frame_timestamp

        segment = self._accumulator.add_frame(frame, frame_timestamp)
        if segment:
            self._push_segment(segment)

    def _idle_timer_callback(self):
        if not self._accumulator.has_frames():
            return

        if (time.time() - self._last_frame_time) >= self._idle_flush_seconds:
            segment = self._accumulator.flush()
            if segment:
                self.get_logger().debug("Flushing partial segment due to inactivity")
                self._push_segment(segment)

    def _push_segment(self, segment: Segment):
        try:
            self._segment_queue.put_nowait(segment)
            self.get_logger().info(
                f"Segment queued ({segment.frame_count} frames, "
                f"duration {segment.duration:.2f}s)"
            )
        except queue.Full:
            self.get_logger().error("Segment queue full, dropping segment")

    def flush_pending(self):
        segment = self._accumulator.flush()
        if segment:
            self.get_logger().info("Flushing remaining frames before shutdown")
            self._push_segment(segment)

    def stop(self):
        self.get_logger().info("Stopping video segmenter node")
        self.flush_pending()
