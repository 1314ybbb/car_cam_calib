#!/usr/bin/env python3
"""Estimate camera optical-center to checkerboard-center distance from one image."""

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np


PATTERN = (9, 6)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", required=True, type=Path, help="full-resolution checkerboard photo")
    parser.add_argument("--intrinsics", required=True, type=Path, help="calibration intrinsics.json")
    parser.add_argument("--square-mm", type=float, help="measured checkerboard square pitch; defaults to calibration value")
    parser.add_argument("--output", type=Path, help="optional result JSON path")
    args = parser.parse_args()

    calibration = json.loads(args.intrinsics.read_text(encoding="utf-8"))
    square_mm = args.square_mm if args.square_mm is not None else float(calibration["square_mm"])
    if square_mm <= 0:
        parser.error("--square-mm must be positive")
    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Cannot read image: {args.image}")
    height, width = image.shape[:2]
    expected = (int(calibration["image_width"]), int(calibration["image_height"]))
    if (width, height) != expected:
        raise RuntimeError(f"Image is {width}x{height}; intrinsics require {expected[0]}x{expected[1]}")

    metadata_path = args.image.with_suffix(".json")
    identity_verified = False
    if metadata_path.is_file():
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        for key in ("camera_model", "camera_serial"):
            if metadata.get(key) != calibration.get(key):
                raise RuntimeError(f"Image {key}={metadata.get(key)!r} differs from calibration {key}={calibration.get(key)!r}")
        identity_verified = True

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCornersSB(gray, PATTERN)
    if not found:
        raise RuntimeError("Complete 9x6 inner corners were not detected")

    grid = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
    grid[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * square_mm
    K = np.asarray(calibration["K"], dtype=np.float64)
    D = np.asarray(calibration["D"], dtype=np.float64)
    success, rotation, translation = cv2.solvePnP(grid, corners, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not success:
        raise RuntimeError("solvePnP could not estimate board pose")
    R, _ = cv2.Rodrigues(rotation)
    # The physical center of a 10x7-square board is also the 9x6-corner grid center.
    board_center_mm = np.array([4.0 * square_mm, 2.5 * square_mm, 0.0])
    center_camera_mm = (R @ board_center_mm.reshape(3, 1) + translation).reshape(3)
    projected, _ = cv2.projectPoints(grid, rotation, translation, K, D)
    residual = projected.reshape(-1, 2) - corners.reshape(-1, 2)
    rms_px = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
    result = {
        "image": str(args.image.resolve()), "intrinsics": str(args.intrinsics.resolve()),
        "camera_model": calibration.get("camera_model"),
        "camera_serial": calibration.get("camera_serial"),
        "identity_verified_from_image_json": identity_verified,
        "square_mm": square_mm, "detected_inner_corners": [9, 6],
        "board_center_camera_xyz_mm": center_camera_mm.tolist(),
        "optical_axis_depth_mm": float(center_camera_mm[2]),
        "optical_center_to_board_center_mm": float(np.linalg.norm(center_camera_mm)),
        "reprojection_rms_px": rms_px,
    }
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"Optical center to board center: {result['optical_center_to_board_center_mm']:.1f} mm")
    print(f"Depth along optical axis:      {result['optical_axis_depth_mm']:.1f} mm")
    print(f"Pose reprojection RMS:         {rms_px:.3f} px")
    if not identity_verified:
        print("Image has no matching metadata JSON; camera identity was not independently checked.")
    if args.output:
        print(f"Saved {args.output}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, cv2.error, OSError, KeyError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
