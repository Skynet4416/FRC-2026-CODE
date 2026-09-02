"""Grabs a single frame from one of the robot's MJPEG camera streams.

PhotonVision (real or simulated) advertises its streams on NetworkTables under
/CameraPublisher/<name>/streams, so the agent can find the cameras without
being told any URLs.
"""

from __future__ import annotations

import urllib.request

_SOI = b"\xff\xd8"  # JPEG start of image
_EOI = b"\xff\xd9"  # JPEG end of image


def grab_frame(url: str, timeout: float = 5.0, max_bytes: int = 4_000_000) -> bytes:
    """Returns the first complete JPEG frame in an MJPEG stream."""
    buffer = b""
    with urllib.request.urlopen(url, timeout=timeout) as stream:
        while len(buffer) < max_bytes:
            chunk = stream.read(4096)
            if not chunk:
                break
            buffer += chunk
            start = buffer.find(_SOI)
            if start < 0:
                continue
            end = buffer.find(_EOI, start + 2)
            if end >= 0:
                return buffer[start : end + 2]
    raise RuntimeError(f"no complete JPEG frame in the first {len(buffer)} bytes of {url}")
