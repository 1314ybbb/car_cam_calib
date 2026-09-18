#!/usr/bin/env python3
"""Show a Hikrobot camera live; Space saves the current full resolution frame."""

import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

from mvs_capture import DEFAULT_MVS, camera_metadata, check, enumerate_cameras, grab_frame, load_sdk, select_camera


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", help="camera serial number")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--mvs-root", type=Path, default=DEFAULT_MVS)
    parser.add_argument("--window-width", type=int, default=1100)
    parser.add_argument("--preview-height", type=int, default=680,
                        help="maximum displayed image height; saved PNG keeps native resolution")
    args = parser.parse_args()
    if args.window_width < 320:
        parser.error("--window-width must be at least 320")
    if args.preview_height < 240:
        parser.error("--preview-height must be at least 240")
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        raise RuntimeError(f"Output directory is not empty: {output}")

    sdk = load_sdk(args.mvs_root)
    check(sdk.MvCamera.MV_CC_Initialize(), "initialize MVS")
    cam = None
    opened = False
    grabbing = False
    original_trigger_mode = None
    try:
        cameras = enumerate_cameras(sdk)
        if not cameras:
            raise RuntimeError("No GigE camera found")
        selected = select_camera(cameras, args.serial)
        print(f"Selected {selected['model']} serial={selected['serial']} "
              f"ip={selected['ip']} host={selected['host_interface_ip']}", flush=True)
        cam = sdk.MvCamera()
        check(cam.MV_CC_CreateHandle(selected["device_info"]), "create camera handle")
        check(cam.MV_CC_OpenDevice(sdk.MV_ACCESS_Exclusive, 0), "open camera")
        opened = True
        trigger = sdk.MVCC_ENUMVALUE()
        if cam.MV_CC_GetEnumValue("TriggerMode", trigger) == 0:
            original_trigger_mode = int(trigger.nCurValue)
        check(cam.MV_CC_SetEnumValue("TriggerMode", sdk.MV_TRIGGER_MODE_OFF), "disable trigger")
        gain_auto = sdk.MVCC_ENUMVALUE()
        if cam.MV_CC_GetEnumValue("GainAuto", gain_auto) == 0 and gain_auto.nCurValue != 0:
            check(cam.MV_CC_SetEnumValue("GainAuto", 0), "disable automatic gain for slider")
        gain_range = sdk.MVCC_FLOATVALUE()
        check(cam.MV_CC_GetFloatValue("Gain", gain_range), "get gain range")
        gain_min = float(gain_range.fMin)
        gain_max = float(gain_range.fMax)
        gain_step = 0.1
        slider_max = max(1, int((gain_max - gain_min) / gain_step))
        gain_state = {"actual": float(gain_range.fCurValue), "pending": None,
                      "ready": False, "last_logged": float(gain_range.fCurValue)}

        def on_gain_slider(position):
            if gain_state["ready"]:
                gain_state["pending"] = position

        settings = camera_metadata(cam, sdk)
        print("Settings:", json.dumps(settings, ensure_ascii=False), flush=True)
        check(cam.MV_CC_StartGrabbing(), "start grabbing")
        grabbing = True
        cv2.namedWindow("Hikrobot live preview", cv2.WINDOW_NORMAL)
        cv2.moveWindow("Hikrobot live preview", 40, 60)
        start_position = int(round((gain_state["actual"] - gain_min) / gain_step))
        start_position = max(0, min(slider_max, start_position))
        cv2.createTrackbar("Gain x0.1", "Hikrobot live preview", start_position,
                           slider_max, on_gain_slider)
        gain_state["ready"] = True
        print(f"Gain slider ready: {gain_min:.2f} to {gain_max:.2f}; "
              "Space saves frame; q or Esc exits.", flush=True)
        count = 0
        last_save = 0.0
        window_fitted = False
        while True:
            pending = gain_state["pending"]
            if pending is not None:
                gain_state["pending"] = None
                requested = min(gain_max, gain_min + pending * gain_step)
                result = cam.MV_CC_SetFloatValue("Gain", requested)
                if result == 0:
                    actual = sdk.MVCC_FLOATVALUE()
                    if cam.MV_CC_GetFloatValue("Gain", actual) == 0:
                        gain_state["actual"] = float(actual.fCurValue)
                        settings["Gain"] = gain_state["actual"]
                        if abs(gain_state["actual"] - gain_state["last_logged"]) >= .5:
                            print(f"Gain set to {gain_state['actual']:.3f}", flush=True)
                            gain_state["last_logged"] = gain_state["actual"]
                else:
                    print(f"WARNING: gain change failed: 0x{result:08x}", flush=True)
            try:
                bgr, frame_info = grab_frame(cam, sdk, 1000)
            except RuntimeError as exc:
                print(exc, flush=True)
                continue
            if frame_info["lost_packets"]:
                print(f"Discarding frame {frame_info['frame_number']}: lost packets", flush=True)
                continue
            scale = min(1.0, args.window_width / bgr.shape[1],
                        args.preview_height / bgr.shape[0])
            preview = cv2.resize(bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            label = (f"{selected['model']}  {bgr.shape[1]}x{bgr.shape[0]}  "
                     f"Gain {gain_state['actual']:.2f}  saved {count}  SPACE save | Q exit")
            cv2.rectangle(preview, (0, 0), (preview.shape[1], 39), (0, 0, 0), -1)
            cv2.putText(preview, label, (10, 27), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.imshow("Hikrobot live preview", preview)
            if not window_fitted:
                cv2.resizeWindow("Hikrobot live preview", preview.shape[1] + 20,
                                 preview.shape[0] + 75)
                cv2.moveWindow("Hikrobot live preview", 40, 60)
                window_fitted = True
            key = cv2.waitKey(1) & 255
            if key in (ord("q"), 27):
                break
            if key != 32 or time.monotonic() - last_save < .4:
                continue
            count += 1
            filename = f"frame_{count:04d}.png"
            image_path = output / filename
            if not cv2.imwrite(str(image_path), bgr):
                raise RuntimeError(f"Failed to save {image_path}")
            intensity = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            metadata = {
                "camera_model": selected["model"], "camera_serial": selected["serial"],
                "camera_ip": selected["ip"], "host_interface_ip": selected["host_interface_ip"],
                "sdk_version": f"0x{sdk.MvCamera.MV_CC_GetSDKVersion():08x}",
                "settings": camera_metadata(cam, sdk), "frame": frame_info,
                "image_stats": {"mean": float(np.mean(intensity)),
                                "min": int(intensity.min()), "max": int(intensity.max())},
                "conversion": "MVS RGB8 then OpenCV BGR8", "capture_unix_ns": time.time_ns(),
            }
            image_path.with_suffix(".json").write_text(json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8")
            last_save = time.monotonic()
            print(f"Saved {image_path}", flush=True)
        print(f"Preview closed; saved {count} frame(s) in {output}", flush=True)
        return 0
    finally:
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        if cam is not None:
            if grabbing:
                cam.MV_CC_StopGrabbing()
            if opened:
                if original_trigger_mode is not None:
                    restore = cam.MV_CC_SetEnumValue("TriggerMode", original_trigger_mode)
                    if restore != 0:
                        print(f"WARNING: could not restore TriggerMode: 0x{restore:08x}", file=sys.stderr)
                cam.MV_CC_CloseDevice()
            cam.MV_CC_DestroyHandle()
        sdk.MvCamera.MV_CC_Finalize()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("Preview stopped.", file=sys.stderr)
        sys.exit(0)
    except (RuntimeError, cv2.error) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
