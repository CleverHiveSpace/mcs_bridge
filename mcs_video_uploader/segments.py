from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np


@dataclass
class Segment:
    """In-memory representation of a video segment ready for encoding."""

    frames: List[np.ndarray]
    timestamps: List[float]

    def __post_init__(self):
        if len(self.frames) != len(self.timestamps):
            raise ValueError("Frames and timestamps must have identical lengths")
        if not self.frames:
            raise ValueError("Segment must contain at least one frame")

    @property
    def frame_count(self) -> int:
        return len(self.frames)

    @property
    def start_time(self) -> float:
        return self.timestamps[0]

    @property
    def end_time(self) -> float:
        return self.timestamps[-1]

    @property
    def duration(self) -> float:
        return max(self.end_time - self.start_time, 1e-6)

    @property
    def resolution(self) -> tuple[int, int]:
        height, width = self.frames[0].shape[:2]
        return width, height

