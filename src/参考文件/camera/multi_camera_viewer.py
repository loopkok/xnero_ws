#!/usr/bin/env python3
"""Multi-camera viewer for Orbbec Gemini 335/305.
Opens ALL connected cameras and shows each in its own window.
Press 'q' to quit, 's' to save all frames.
"""
import os, sys, time
import numpy as np
import cv2

from pyorbbecsdk import (
    Pipeline, Config, Context,
    OBSensorType, OBStreamType, OBFormat,
)


def get_device_list():
    """Enumerate all connected Orbbec devices."""
    ctx = Context()
    dev_list = ctx.query_devices()
    n = dev_list.get_count()
    devices = []
    for i in range(n):
        devices.append({
            "serial": dev_list.get_device_serial_number_by_index(i),
            "name": dev_list.get_device_name_by_index(i),
            "pid": dev_list.get_device_pid_by_index(i),
            "uid": dev_list.get_device_uid_by_index(i),
        })
    return devices


def main():
    devices = get_device_list()
    if not devices:
        print("No Orbbec devices found!")
        return

    print(f"Found {len(devices)} device(s):")
    for i, d in enumerate(devices):
        print(f"  [{i}] {d['name']}  SN={d['serial']}  PID={d['pid']}  UID={d['uid']}")

    # Get Device objects by UID and create per-device pipelines
    ctx = Context()
    pipelines = []
    for i, dev_info in enumerate(devices):
        d = ctx.query_devices().get_device_by_uid(dev_info["uid"])
        pipeline = Pipeline(d)
        config = Config()
        color_profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        try:
            profile = color_profiles.get_video_stream_profile(640, 480, OBFormat.MJPG, 30)
        except:
            profile = color_profiles.get_default_video_stream_profile()
        config.enable_stream(profile)
        pipeline.start(config)
        pipelines.append(pipeline)
        print(f"  [{i}] {dev_info['name']} started @ 640x480")

    if not pipelines:
        print("No pipelines started.")
        return

    win_names = [f"[{i}] {d['name']}  SN={d['serial'][-4:]}" for i, d in enumerate(devices)]
    for name in win_names:
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(name, 640, 480)

    print("\n  [q] quit  [s] save all frames\n")

    try:
        while True:
            for i, pipe in enumerate(pipelines):
                try:
                    frames = pipe.wait_for_frames(100)
                    if frames is None:
                        continue
                    color_frame = frames.get_color_frame()
                    if color_frame is None:
                        continue
                    data = np.asanyarray(color_frame.get_data())
                    w, h = color_frame.get_width(), color_frame.get_height()
                    fmt = color_frame.get_format()

                    if fmt in (OBFormat.MJPG,):
                        img = cv2.imdecode(data, cv2.IMREAD_COLOR)
                    elif fmt == OBFormat.RGB:
                        img = data.reshape((h, w, 3))
                        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    elif fmt == OBFormat.BGR:
                        img = data.reshape((h, w, 3))
                    else:
                        img = data.reshape((h, w, 3))

                    if img is not None:
                        cv2.imshow(win_names[i], img)
                except Exception:
                    pass

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
            elif key == ord("s"):
                ts = int(time.time())
                save_dir = f"/tmp/camera_capture_{ts}"
                os.makedirs(save_dir, exist_ok=True)
                for i, pipe in enumerate(pipelines):
                    frames = pipe.wait_for_frames(100)
                    if frames:
                        cf = frames.get_color_frame()
                        if cf:
                            d = np.asanyarray(cf.get_data())
                            w, h = cf.get_width(), cf.get_height()
                            if cf.get_format() in (OBFormat.MJPG,):
                                img = cv2.imdecode(d, cv2.IMREAD_COLOR)
                            else:
                                img = d.reshape((h, w, 3))
                            path = os.path.join(save_dir, f"cam{i}_{devices[i]['name']}_{devices[i]['serial']}.jpg")
                            cv2.imwrite(path, img)
                print(f"Saved to {save_dir}/")

    except KeyboardInterrupt:
        pass
    finally:
        for pipe in pipelines:
            pipe.stop()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()
