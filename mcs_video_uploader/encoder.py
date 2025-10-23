import logging
import os
import shutil
import subprocess
import tempfile
from typing import Optional

import numpy as np

from .config import (
    FFMPEG_PATH,
    VIDEO_H264_BITRATE,
    VIDEO_H264_CRF,
    VIDEO_H264_MOVFLAGS,
    VIDEO_H264_PIX_FMT,
    VIDEO_H264_PRESET,
    VIDEO_H264_PROFILE,
    VIDEO_H264_TUNE,
    VIDEO_TARGET_FPS,
    VIDEO_TMP_DIR,
)
from .segments import Segment

logger = logging.getLogger(__name__)


class SegmentEncodingError(RuntimeError):
    """Raised when a segment fails to encode."""


def _compute_effective_fps(segment: Segment, preferred_fps: float) -> float:
    if segment.frame_count <= 1:
        return max(preferred_fps, 1.0) if preferred_fps > 0 else 1.0

    observed_fps = (segment.frame_count - 1) / segment.duration
    if preferred_fps > 0:
        return max(1.0, min(observed_fps, preferred_fps))
    return max(1.0, observed_fps)


def _resolve_ffmpeg(path: str) -> str:
    executable = shutil.which(path)
    if not executable:
        raise SegmentEncodingError(
            f"ffmpeg binary '{path}' not found in PATH. Set VIDEO_FFMPEG_PATH or install ffmpeg."
        )
    return executable


def _build_ffmpeg_command(
    *,
    ffmpeg_bin: str,
    width: int,
    height: int,
    fps: float,
    output_path: str,
) -> list[str]:
    cmd = [
        ffmpeg_bin,
        "-y",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "bgr24",
        "-s",
        f"{width}x{height}",
        "-r",
        f"{fps:.6f}",
        "-i",
        "pipe:0",
        "-c:v",
        "libx264",
        "-preset",
        VIDEO_H264_PRESET,
        "-profile:v",
        VIDEO_H264_PROFILE,
        "-crf",
        str(VIDEO_H264_CRF),
        "-pix_fmt",
        VIDEO_H264_PIX_FMT,
    ]

    if VIDEO_H264_TUNE:
        cmd.extend(["-tune", VIDEO_H264_TUNE])
    if VIDEO_H264_BITRATE:
        cmd.extend(["-b:v", VIDEO_H264_BITRATE])
    if VIDEO_H264_MOVFLAGS:
        cmd.extend(["-movflags", VIDEO_H264_MOVFLAGS])

    cmd.extend(["-f", "mp4", output_path])
    return cmd


def encode_segment_to_h264(
    segment: Segment,
    preferred_fps: float = VIDEO_TARGET_FPS,
    tmp_dir: Optional[str] = VIDEO_TMP_DIR,
) -> bytes:
    """Encode the provided segment into an MP4 container using ffmpeg/libx264."""
    ffmpeg_bin = _resolve_ffmpeg(FFMPEG_PATH)
    fps = _compute_effective_fps(segment, preferred_fps)
    width, height = segment.resolution

    tmp_file = tempfile.NamedTemporaryFile(
        suffix=".mp4",
        prefix="segment-",
        dir=tmp_dir or None,
        delete=False,
    )
    tmp_path = tmp_file.name
    tmp_file.close()

    cmd = _build_ffmpeg_command(
        ffmpeg_bin=ffmpeg_bin,
        width=width,
        height=height,
        fps=fps,
        output_path=tmp_path,
    )

    logger.debug("Encoding segment via ffmpeg: %s", " ".join(cmd))

    process = subprocess.Popen(
        cmd,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=0,
    )

    stdout_data = b""
    stderr_data = b""

    try:
        assert process.stdin is not None
        for frame in segment.frames:
            if frame.shape[1] != width or frame.shape[0] != height:
                raise SegmentEncodingError("All frames must share identical resolution")
            if frame.dtype != np.uint8:
                raise SegmentEncodingError("Frames must be 8-bit images")
            if frame.ndim != 3 or frame.shape[2] != 3:
                raise SegmentEncodingError("Frames must be color images with 3 channels (BGR)")

            contiguous = np.ascontiguousarray(frame)
            try:
                process.stdin.write(contiguous.tobytes())
            except BrokenPipeError as exc:  # pragma: no cover - defensive
                raise SegmentEncodingError("ffmpeg process ended unexpectedly") from exc

        process.stdin.close()
        process.stdin = None
        stdout_data, stderr_data = process.communicate()
    finally:
        if process.stdin and not process.stdin.closed:
            process.stdin.close()

    if process.returncode != 0:
        stderr_text = stderr_data.decode("utf-8", errors="replace") if stderr_data else ""
        stdout_text = stdout_data.decode("utf-8", errors="replace") if stdout_data else ""
        raise SegmentEncodingError(
            f"ffmpeg failed with exit code {process.returncode}.\n"
            f"stdout:\n{stdout_text}\n"
            f"stderr:\n{stderr_text}"
        )

    try:
        with open(tmp_path, "rb") as handle:
            payload = handle.read()
    finally:
        try:
            os.remove(tmp_path)
        except OSError:
            logger.warning("Failed to remove temporary file %s", tmp_path)

    logger.info(
        "Encoded segment with %d frames (duration %.2fs, fps %.2f) using ffmpeg/libx264",
        segment.frame_count,
        segment.duration,
        fps,
    )

    return payload
