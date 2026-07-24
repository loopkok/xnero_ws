#!/usr/bin/env python3
"""Stream Gemini 335 camera feed to Quest3 HTS Video Feed via WebRTC.

Usage:
    pip install hand-tracking-sdk av opencv-python
    python3 gemini_video_stream.py                          # default 1280x720
    python3 gemini_video_stream.py --width 640 --height 480
    python3 gemini_video_stream.py --preset 720p --port 8765

Quest3 side: open HTS → Video Feed → connect to ws://<PC_IP>:8765
"""

from __future__ import annotations

import argparse
import asyncio
import sys
from typing import Any

import cv2
import numpy as np

# --- Gemini 335 capture (same SDK v2 API) ---
from pyorbbecsdk import (
    Pipeline, Config, Context, OBSensorType, OBFormat,
)

# --- HTS video streaming ---
from hand_tracking_sdk.video.sources import VideoSourceAdapter, VideoFormat
from hand_tracking_sdk.video.service import VideoService, VideoServiceConfig


class GeminiSourceAdapter(VideoSourceAdapter):
    """Video source that reads frames from an Orbbec Gemini 335/305 camera."""

    def __init__(self, *, width: int = 1280, height: int = 720, fps: int = 30):
        self._format = VideoFormat(width=width, height=height, fps=fps)
        self._pipeline: Any = None

    async def start(self) -> None:
        """Open the Gemini 335 camera - color stream only (no depth)."""
        ctx = Context()
        dev_list = ctx.query_devices()
        print(f"Found {dev_list.get_count()} Orbbec device(s).")

        # Pick the Gemini 335
        device = None
        for i in range(dev_list.get_count()):
            name = dev_list.get_device_name_by_index(i)
            uid = dev_list.get_device_uid_by_index(i)
            sn = dev_list.get_device_serial_number_by_index(i)
            print(f"  [{i}] {name}  SN={sn}  UID={uid}")
            if "335" in name:
                device = dev_list.get_device_by_uid(uid)

        if device is None:
            raise RuntimeError("Gemini 335 not found.")

        pipeline = Pipeline(device)
        config = Config()
        # Color stream only - prefer MJPG for lower bandwidth
        color_profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        try:
            profile = color_profiles.get_video_stream_profile(
                self._format.width, self._format.height, OBFormat.MJPG, self._format.fps
            )
        except Exception:
            profile = color_profiles.get_default_video_stream_profile()
        config.enable_stream(profile)
        pipeline.start(config)
        self._pipeline = pipeline
        print(f"Streaming {self._format.width}x{self._format.height} @ {self._format.fps}fps")

    async def stop(self) -> None:
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None

    def get_format(self) -> VideoFormat:
        return self._format

    async def next_frame(self) -> Any:
        import av

        if self._pipeline is None:
            raise RuntimeError("Camera not started.")

        while True:
            frames = self._pipeline.wait_for_frames(100)
            if frames is None:
                await asyncio.sleep(0.001)
                continue
            color_frame = frames.get_color_frame()
            if color_frame is None:
                await asyncio.sleep(0.001)
                continue

            data = np.asanyarray(color_frame.get_data())
            fmt = color_frame.get_format()

            if fmt == OBFormat.MJPG:
                img = cv2.imdecode(data, cv2.IMREAD_COLOR)
                rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            else:
                w, h = color_frame.get_width(), color_frame.get_height()
                if fmt == OBFormat.RGB:
                    rgb = data.reshape((h, w, 3))
                else:
                    rgb = cv2.cvtColor(data.reshape((h, w, 3)), cv2.COLOR_BGR2RGB)

            return av.VideoFrame.from_ndarray(rgb, format="rgb24")


async def _run(args: argparse.Namespace) -> int:
    source = GeminiSourceAdapter(
        width=args.width,
        height=args.height,
        fps=args.fps,
    )
    config = VideoServiceConfig(
        signaling_host=args.host,
        signaling_port=args.port,
        source="gemini",  # custom source, will be patched
        preset=args.preset,
        verbose=args.verbose,
    )
    service = VideoService(config)
    # Patch: inject our custom adapter
    service._build_source = lambda: source
    service._VALID_SOURCES = service._VALID_SOURCES + ("gemini",)

    print(f"Signaling endpoint: ws://<HOST_IP>:{args.port}")
    print("Quest3 HTS → Video Feed → connect to the above address")
    await service.start()
    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        await service.stop()
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(description="Stream Gemini camera to Quest3 HTS")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--host", default="0.0.0.0", help="Signaling bind host")
    parser.add_argument("--port", type=int, default=8765, help="Signaling bind port")
    parser.add_argument("--preset", default="720p", help="Video encoding preset")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    try:
        raise SystemExit(asyncio.run(_run(args)))
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
