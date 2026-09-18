#!/usr/bin/env python3
"""Capture calibration images at the camera's current resolution and ROI."""

import argparse
import ctypes
import ipaddress
import json
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np


DEFAULT_MVS = Path.home() / ".local/opt/MVS-5.0.1"


def load_sdk(root):
    root = Path(root).expanduser().resolve()
    sample = root / "Samples/64/Python/MvImport"
    library = root / "lib/64"
    if not sample.is_dir() or not (library / "libMvCameraControl.so").exists():
        raise RuntimeError(f"MVS SDK not found at {root}")
    os.environ["MVCAM_COMMON_RUNENV"] = str(root / "lib")
    os.environ["LD_LIBRARY_PATH"] = str(library) + ":" + os.environ.get("LD_LIBRARY_PATH", "")
    # The SDK opens transport libraries later; an RPATH-like pre-load helps when
    # MVS was unpacked locally instead of installed system-wide.
    ctypes.CDLL(str(library / "libMvCameraControl.so"), mode=ctypes.RTLD_GLOBAL)
    sys.path.insert(0, str(sample))
    import MvCameraControl_class as sdk  # type: ignore
    return sdk


def check(code, operation):
    if code != 0:
        raise RuntimeError(f"{operation} failed: 0x{code:08x}")


def decode(chars):
    return bytes(chars).split(b"\0", 1)[0].decode("utf-8", errors="replace")


def ip_address(value):
    return ".".join(str((value >> shift) & 255) for shift in (24, 16, 8, 0))


def enumerate_cameras(sdk):
    devices = sdk.MV_CC_DEVICE_INFO_LIST()
    check(sdk.MvCamera.MV_CC_EnumDevices(sdk.MV_GIGE_DEVICE, devices), "enumerate GigE cameras")
    cameras = []
    for index in range(devices.nDeviceNum):
        info = ctypes.cast(devices.pDeviceInfo[index], ctypes.POINTER(sdk.MV_CC_DEVICE_INFO)).contents
        gig = info.SpecialInfo.stGigEInfo
        cameras.append({
            "index": index,
            "model": decode(gig.chModelName),
            "serial": decode(gig.chSerialNumber),
            "ip": ip_address(gig.nCurrentIp),
            "host_interface_ip": ip_address(gig.nNetExport),
            "subnet_mask": ip_address(gig.nCurrentSubNetMask),
            "device_info": info,
        })
    return cameras


def camera_metadata(cam, sdk):
    values = {}
    for key in ("Width", "Height", "OffsetX", "OffsetY", "BinningHorizontal", "BinningVertical", "DecimationHorizontal", "DecimationVertical"):
        item = sdk.MVCC_INTVALUE_EX()
        if cam.MV_CC_GetIntValueEx(key, item) == 0:
            values[key] = int(item.nCurValue)
    for key in ("PixelFormat", "TriggerMode", "ExposureAuto", "GainAuto", "BalanceWhiteAuto"):
        item = sdk.MVCC_ENUMVALUE()
        if cam.MV_CC_GetEnumValue(key, item) == 0:
            values[key] = int(item.nCurValue)
    for key in ("ExposureTime", "Gain"):
        item = sdk.MVCC_FLOATVALUE()
        if cam.MV_CC_GetFloatValue(key, item) == 0:
            values[key] = float(item.fCurValue)
    return values


def frame_to_bgr(cam, sdk, frame):
    info = frame.stFrameInfo
    width, height = int(info.nWidth), int(info.nHeight)
    if width <= 0 or height <= 0:
        raise RuntimeError("SDK returned invalid frame dimensions")
    # MVS conversion is used for Bayer and packed pixel formats. Keep the same
    # conversion path for every saved calibration image.
    target = sdk.PixelType_Gvsp_RGB8_Packed
    output = (ctypes.c_ubyte * (width * height * 3))()
    conversion = sdk.MV_CC_PIXEL_CONVERT_PARAM_EX()
    conversion.nWidth = width
    conversion.nHeight = height
    conversion.enSrcPixelType = info.enPixelType
    conversion.pSrcData = frame.pBufAddr
    conversion.nSrcDataLen = info.nFrameLen
    conversion.enDstPixelType = target
    conversion.pDstBuffer = output
    conversion.nDstBufferSize = ctypes.sizeof(output)
    check(cam.MV_CC_ConvertPixelTypeEx(conversion), "convert frame to RGB8")
    if conversion.nDstLen < width * height * 3:
        raise RuntimeError("converted frame is shorter than expected")
    rgb = np.frombuffer(output, np.uint8, count=width * height * 3).reshape(height, width, 3)
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def grab_frame(cam, sdk, timeout_ms=1000):
    frame = sdk.MV_FRAME_OUT()
    result = cam.MV_CC_GetImageBuffer(frame, timeout_ms)
    if result != 0:
        raise RuntimeError(f"Frame timeout/error: 0x{result:08x}")
    try:
        if not frame.pBufAddr:
            raise RuntimeError("SDK returned a null frame buffer")
        bgr = frame_to_bgr(cam, sdk, frame)
        info = frame.stFrameInfo
        metadata = {"frame_number": int(info.nFrameNum),
                    "pixel_type": int(info.enPixelType),
                    "lost_packets": int(info.nLostPacket),
                    "device_timestamp_raw": (int(info.nDevTimeStampHigh) << 32) | int(info.nDevTimeStampLow),
                    "width": int(info.nWidth), "height": int(info.nHeight)}
        return bgr, metadata
    finally:
        check(cam.MV_CC_FreeImageBuffer(frame), "release frame")


def select_camera(cameras, serial=None, index=None):
    matches = [c for c in cameras if (serial is None or c["serial"] == serial)
               and (index is None or c["index"] == index)]
    if not matches:
        raise RuntimeError("No camera matches --serial/--index")
    if len({c["serial"] for c in matches}) > 1:
        raise RuntimeError("Multiple physical cameras found; select one with --serial")
    for entry in matches:
        try:
            network = ipaddress.ip_network(f"{entry['ip']}/{entry['subnet_mask']}", strict=False)
            if ipaddress.ip_address(entry["host_interface_ip"]) in network:
                return entry
        except ValueError:
            pass
    return matches[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("probe", "snapshot", "capture"))
    parser.add_argument("--mvs-root", type=Path, default=DEFAULT_MVS)
    parser.add_argument("--serial", help="required for capture when multiple cameras are present")
    parser.add_argument("--index", type=int, help="specific SDK enumeration index, for diagnostics")
    parser.add_argument("--output", type=Path, help="PNG path for snapshot; new image directory for capture")
    parser.add_argument("--expected-model", default="MV-CS050-10GC",
                        help="capture only: exact model to allow; default MV-CS050-10GC")
    parser.add_argument("--max-images", type=int, default=35)
    args = parser.parse_args()
    if args.command in ("snapshot", "capture") and not args.output:
        parser.error(f"{args.command} requires --output")
    if args.max_images < 1:
        parser.error("--max-images must be positive")

    sdk = load_sdk(args.mvs_root)
    check(sdk.MvCamera.MV_CC_Initialize(), "initialize MVS")
    cam = None
    opened = False
    grabbing = False
    original_trigger_mode = None
    try:
        print(f"MVS SDK version: 0x{sdk.MvCamera.MV_CC_GetSDKVersion():08x}")
        cameras = enumerate_cameras(sdk)
        for entry in cameras:
            print(f"[{entry['index']}] {entry['model']} serial={entry['serial']} "
                  f"ip={entry['ip']} host={entry['host_interface_ip']}")
        if not cameras:
            print("No GigE camera found. Check power, Ethernet link, camera/host IP, then rerun probe.")
            return 2
        if args.command == "probe":
            return 0
        selected = select_camera(cameras, args.serial, args.index)
        if args.command == "capture" and selected["model"] != args.expected_model:
            raise RuntimeError(f"Camera reports {selected['model']}, expected {args.expected_model}; "
                               "verify the camera or set --expected-model explicitly")
        output = args.output.expanduser().resolve()
        if args.command == "capture":
            output.mkdir(parents=True, exist_ok=True)
            if any(output.iterdir()):
                raise RuntimeError(f"Output directory is not empty: {output}")
        else:
            if output.suffix.lower() != ".png":
                raise RuntimeError("Snapshot output must be a .png file")
            output.parent.mkdir(parents=True, exist_ok=True)
            if output.exists():
                raise RuntimeError(f"Snapshot already exists: {output}")
        cam = sdk.MvCamera()
        check(cam.MV_CC_CreateHandle(selected["device_info"]), "create camera handle")
        print(f"Selected index {selected['index']} via host {selected['host_interface_ip']}")
        check(cam.MV_CC_OpenDevice(sdk.MV_ACCESS_Exclusive, 0), "open camera")
        opened = True
        # Free running acquisition is needed for the interactive viewfinder.
        trigger_value = sdk.MVCC_ENUMVALUE()
        if cam.MV_CC_GetEnumValue("TriggerMode", trigger_value) == 0:
            original_trigger_mode = int(trigger_value.nCurValue)
        check(cam.MV_CC_SetEnumValue("TriggerMode", sdk.MV_TRIGGER_MODE_OFF), "disable trigger")
        settings = camera_metadata(cam, sdk)
        print("Camera settings:", json.dumps(settings, ensure_ascii=False))
        check(cam.MV_CC_StartGrabbing(), "start grabbing")
        grabbing = True
        if args.command == "snapshot":
            for attempt in range(10):
                try:
                    bgr, frame_metadata = grab_frame(cam, sdk)
                except RuntimeError as exc:
                    print(f"Attempt {attempt + 1}/10: {exc}")
                    continue
                if frame_metadata["lost_packets"]:
                    print(f"Attempt {attempt + 1}/10: {frame_metadata['lost_packets']} lost packets")
                    continue
                if not cv2.imwrite(str(output), bgr):
                    raise RuntimeError(f"Failed to save {output}")
                intensity = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                image_stats = {"min": int(intensity.min()), "max": int(intensity.max()),
                               "mean": float(intensity.mean()),
                               "p99": float(np.percentile(intensity, 99))}
                snapshot_metadata = {"camera_model": selected["model"],
                                     "camera_serial": selected["serial"],
                                     "camera_ip": selected["ip"],
                                     "host_interface_ip": selected["host_interface_ip"],
                                     "sdk_version": f"0x{sdk.MvCamera.MV_CC_GetSDKVersion():08x}",
                                     "conversion": "MVS RGB8 then OpenCV BGR8",
                                     "settings": settings, "frame": frame_metadata,
                                     "image_stats": image_stats,
                                     "capture_unix_ns": time.time_ns()}
                metadata_path = output.with_suffix(".json")
                metadata_path.write_text(json.dumps(snapshot_metadata, indent=2, ensure_ascii=False), encoding="utf-8")
                print(f"Saved {output}: {bgr.shape[1]}x{bgr.shape[0]}, "
                      f"pixel_type={frame_metadata['pixel_type']}, lost_packets=0")
                if image_stats["p99"] < 5:
                    print("WARNING: image is almost black; check lens cap, aperture, lighting and exposure")
                return 0
            raise RuntimeError("Could not obtain a complete frame in 10 attempts")
        print("Space: save sharp, varied board pose; q/Esc: finish. Keep board steady.")
        count = 0
        last_saved = 0.0
        manifest = {
            "camera_model": selected["model"], "camera_serial": selected["serial"],
            "camera_ip": selected["ip"], "sdk_version": f"0x{sdk.MvCamera.MV_CC_GetSDKVersion():08x}",
            "conversion": "MVS RGB8 then OpenCV BGR8", "settings": settings,
            "board": {"squares": [10, 7], "inner_corners": [9, 6], "square_mm": 50},
            "images": [],
        }
        while count < args.max_images:
            try:
                bgr, frame_metadata = grab_frame(cam, sdk)
            except RuntimeError as exc:
                print(exc)
                continue
            frame_num = frame_metadata["frame_number"]
            pixel_type = frame_metadata["pixel_type"]
            lost_packets = frame_metadata["lost_packets"]
            if lost_packets:
                print(f"Skipping frame {frame_num}: {lost_packets} lost packet(s)")
                continue
            preview = cv2.resize(bgr, None, fx=min(1.0, 1100 / bgr.shape[1]), fy=min(1.0, 1100 / bgr.shape[1]))
            cv2.putText(preview, f"saved {count}/{args.max_images} | space=save q=quit", (12, 32), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 0), 2)
            cv2.imshow("MVS calibration capture", preview)
            key = cv2.waitKey(1) & 255
            if key in (ord("q"), 27):
                break
            if key != 32 or time.monotonic() - last_saved < .5:
                continue
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCornersSB(gray, (9, 6))
            if not found:
                print("Board not fully visible/detected; image not saved")
                continue
            count += 1
            name = f"{count:03d}.png"
            if not cv2.imwrite(str(output / name), bgr):
                raise RuntimeError(f"Failed to save {name}")
            manifest["images"].append({"file": name, "frame_number": frame_num,
                                       "pixel_type": pixel_type, "lost_packets": lost_packets,
                                       "capture_unix_ns": time.time_ns(),
                                       "board_center_px": np.mean(corners.reshape(-1, 2), axis=0).tolist()})
            (output / "capture_manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
            last_saved = time.monotonic()
            print(f"Saved {output / name}")
        print(f"Captured {count} images in {output}")
        return 0
    finally:
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            # A headless OpenCV build can still run `probe`.
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
    except (RuntimeError, cv2.error) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
