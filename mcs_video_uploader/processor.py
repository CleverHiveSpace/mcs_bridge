import logging
import queue
import threading

from .api import VideoClient
from .encoder import SegmentEncodingError, encode_segment_to_h264
from .segments import Segment

logger = logging.getLogger(__name__)


class SegmentProcessor(threading.Thread):
    """Background worker that encodes queued segments and streams them to the backend."""

    def __init__(
        self,
        *,
        client: VideoClient,
        camera_id: str,
        segment_queue: queue.Queue,
        ack_timeout: float,
    ):
        super().__init__(daemon=True)
        self._client = client
        self._camera_id = camera_id
        self._queue = segment_queue
        self._ack_timeout = ack_timeout
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):
        logger.info("Segment processor started")
        while not self._stop_event.is_set() or not self._queue.empty():
            try:
                segment: Segment = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue

            try:
                self._process_segment(segment)
            except SegmentEncodingError as exc:
                logger.error("Encoding failed: %s", exc)
            except TimeoutError as exc:
                logger.error("Timed out delivering segment: %s", exc)
            except Exception:  # noqa: BLE001
                logger.exception("Unexpected error while handling segment")
            finally:
                self._queue.task_done()
        logger.info("Segment processor stopped")

    def _process_segment(self, segment: Segment):
        video_bytes = encode_segment_to_h264(segment)

        start_ms = int(segment.start_time * 1000)
        end_ms = int(segment.end_time * 1000)
        file_name = f"{self._camera_id}-{start_ms}-{end_ms}.mp4"

        payload = {
            "cameraId": self._camera_id,
            "startDate": start_ms,
            "endDate": end_ms,
            "chunk": video_bytes,
            "mimeType": "video/mp4",
            "fileName": file_name,
        }

        logger.debug(
            "Sending segment %s (%d frames, %.2fs)",
            file_name,
            segment.frame_count,
            segment.duration,
        )

        response = self._client.send_segment(payload, timeout=self._ack_timeout)

        if isinstance(response, dict):
            if response.get("ok"):
                logger.info(
                    "Segment %s stored successfully (id=%s)",
                    file_name,
                    response.get("id"),
                )
            else:
                error_message = response.get("error", "Unknown error")
                logger.error("Segment %s rejected: %s", file_name, error_message)
        else:
            logger.warning("Segment acknowledgement was not a dict: %s", response)
