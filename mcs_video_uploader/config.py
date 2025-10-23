import os
from pathlib import Path

from dotenv import load_dotenv


env_path = Path(__file__).parent.parent / ".env"
load_dotenv(env_path, override=True)


BACKEND_URL = os.getenv("BACKEND_URL", "http://localhost:1234")
VIDEO_WS_NAMESPACE = os.getenv("VIDEO_WS_NAMESPACE", "/ws/video")

USERNAME = os.getenv("USERNAME")
PASSWORD = os.getenv("PASSWORD")

VIDEO_TOPIC = os.getenv("VIDEO_TOPIC", "/navcam_front/image_raw/image_color")
VIDEO_CAMERA_ID = os.getenv("VIDEO_CAMERA_ID", "camera-1")
VIDEO_SEGMENT_SECONDS = float(os.getenv("VIDEO_SEGMENT_SECONDS", "5"))
VIDEO_TARGET_FPS = float(os.getenv("VIDEO_TARGET_FPS", "30"))
VIDEO_IDLE_FLUSH_SECONDS = float(os.getenv("VIDEO_IDLE_FLUSH_SECONDS", "2"))
VIDEO_MAX_QUEUE = int(os.getenv("VIDEO_MAX_QUEUE", "4"))
VIDEO_TMP_DIR = os.getenv("VIDEO_TMP_DIR")
VIDEO_ACK_TIMEOUT = float(os.getenv("VIDEO_ACK_TIMEOUT", "10"))

FFMPEG_PATH = os.getenv("VIDEO_FFMPEG_PATH", "ffmpeg")
VIDEO_H264_PRESET = os.getenv("VIDEO_H264_PRESET", "veryfast")
VIDEO_H264_PROFILE = os.getenv("VIDEO_H264_PROFILE", "baseline")
VIDEO_H264_TUNE = os.getenv("VIDEO_H264_TUNE", "")
VIDEO_H264_CRF = int(os.getenv("VIDEO_H264_CRF", "23"))
VIDEO_H264_BITRATE = os.getenv("VIDEO_H264_BITRATE")
VIDEO_H264_MOVFLAGS = os.getenv("VIDEO_H264_MOVFLAGS", "+faststart")
VIDEO_H264_PIX_FMT = os.getenv("VIDEO_H264_PIX_FMT", "yuv420p")
