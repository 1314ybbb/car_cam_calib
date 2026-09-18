#!/usr/bin/env python3
"""Estimate pinhole + 5 coefficient distortion intrinsics from a 9x6 chessboard."""

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np


PATTERN = (9, 6)  # inner corners, columns first
EXTENSIONS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


def images_in(folder):
    if not folder.is_dir():
        raise RuntimeError(f"Image directory does not exist: {folder}")
    return sorted(p for p in folder.iterdir() if p.suffix.lower() in EXTENSIONS)


def detect(folder, expected_size=None, annotated=None):
    records = []
    failures = []
    size = expected_size
    paths = images_in(folder)
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            failures.append({"file": path.name, "reason": "unreadable"})
            continue
        height, width = image.shape[:2]
        if size is None:
            size = (width, height)
        if (width, height) != size:
            failures.append({"file": path.name, "reason": f"size {width}x{height} differs from {size[0]}x{size[1]}"})
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(gray, PATTERN)
        if not found:
            failures.append({"file": path.name, "reason": "9x6 corners not detected"})
            continue
        pts = corners.reshape(-1, 2)
        hull_area = cv2.contourArea(cv2.convexHull(pts))
        records.append({"file": path.name, "path": path, "corners": corners.astype(np.float32),
                        "board_center_px": np.mean(pts, axis=0).tolist(),
                        "board_area_fraction": float(hull_area / (width * height)),
                        "sharpness_laplacian_variance": float(cv2.Laplacian(gray, cv2.CV_64F).var())})
        if annotated:
            view = image.copy()
            cv2.drawChessboardCorners(view, PATTERN, corners, True)
            cv2.imwrite(str(annotated / path.name), view)
    return records, failures, size, len(paths)


def object_grid(square_mm):
    grid = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
    grid[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * square_mm
    return grid


def fit(records, size, grid, flags=0):
    object_points = [grid.copy() for _ in records]
    image_points = [record["corners"] for record in records]
    rms, K, D, rvecs, tvecs, std_i, _, per_view = cv2.calibrateCameraExtended(
        object_points, image_points, size, None, None,
        flags=flags, criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-10))
    if not np.isfinite(K).all() or not np.isfinite(D).all():
        raise RuntimeError("Calibration produced non-finite parameters")
    return float(rms), K, D.reshape(-1), rvecs, tvecs, std_i.reshape(-1), per_view.reshape(-1)


def validation_errors(records, grid, K, D):
    results = []
    for record in records:
        ok, rotation, translation = cv2.solvePnP(grid, record["corners"], K, D)
        if not ok:
            results.append({"file": record["file"], "error": "solvePnP failed"})
            continue
        projected, _ = cv2.projectPoints(grid, rotation, translation, K, D)
        residual = projected.reshape(-1, 2) - record["corners"].reshape(-1, 2)
        results.append({"file": record["file"],
                        "reprojection_rms_px": float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))})
    return results


def ros_yaml(name, size, K, D):
    flat_k = K.reshape(-1).tolist()
    p = [K[0, 0], K[0, 1], K[0, 2], 0.0,
         K[1, 0], K[1, 1], K[1, 2], 0.0,
         0.0, 0.0, 1.0, 0.0]
    values = lambda items: ", ".join(f"{float(item):.12g}" for item in items)
    return (f"image_width: {size[0]}\nimage_height: {size[1]}\n"
            f"camera_name: {name}\ndistortion_model: plumb_bob\n"
            f"camera_matrix:\n  rows: 3\n  cols: 3\n  data: [{values(flat_k)}]\n"
            f"distortion_coefficients:\n  rows: 1\n  cols: 5\n  data: [{values(D)}]\n"
            f"rectification_matrix:\n  rows: 3\n  cols: 3\n  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n"
            f"projection_matrix:\n  rows: 3\n  cols: 4\n  data: [{values(p)}]\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--square-mm", type=float, default=50.0,
                        help="measured physical square pitch in mm; default 50")
    parser.add_argument("--validation-images", type=Path,
                        help="separate images taken after fitting to check reprojection")
    parser.add_argument("--fix-k3", action="store_true",
                        help="fit k1,k2,p1,p2 while fixing the poorly constrained k3 term to zero")
    args = parser.parse_args()
    if args.square_mm <= 0:
        parser.error("--square-mm must be positive")
    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    annotations = output / "detected_corners"
    annotations.mkdir(exist_ok=True)
    records, failures, size, total = detect(args.images.expanduser().resolve(), annotated=annotations)
    if len(records) < 15:
        raise RuntimeError(f"Only {len(records)} valid views of {total}; need at least 15, aim for 25-35 varied views")
    grid = object_grid(args.square_mm)
    rms, K, D, rvecs, tvecs, std_i, per_view = fit(
        records, size, grid, flags=cv2.CALIB_FIX_K3 if args.fix_k3 else 0)
    for record, error in zip(records, per_view):
        record["reprojection_rms_px"] = float(error)
    validation = None
    if args.validation_images:
        valid, validation_failures, _, validation_total = detect(args.validation_images.expanduser().resolve(), expected_size=size)
        validation = {"directory": str(args.validation_images.expanduser().resolve()),
                      "total_images": validation_total, "detected_images": len(valid),
                      "failed_images": validation_failures,
                      "per_image": validation_errors(valid, grid, K, D)}
    manifest_path = args.images.expanduser().resolve() / "capture_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else {}
    image_source = "mvs_capture" if manifest else "external_images_unverified"
    if not manifest:
        metadata_paths = [record["path"].with_suffix(".json") for record in records]
        if all(path.is_file() for path in metadata_paths):
            metadata = [json.loads(path.read_text(encoding="utf-8")) for path in metadata_paths]
            models = {item.get("camera_model") for item in metadata}
            serials = {item.get("camera_serial") for item in metadata}
            if len(models) != 1 or len(serials) != 1:
                raise RuntimeError("Preview image metadata contains multiple camera models or serial numbers")
            manifest = {"camera_model": metadata[0].get("camera_model"),
                        "camera_serial": metadata[0].get("camera_serial"),
                        "settings": metadata[0].get("settings")}
            image_source = metadata[0].get("capture_app", "live_preview")
    serial = manifest.get("camera_serial", "unknown")
    centers = np.array([record["board_center_px"] for record in records])
    all_corners = np.concatenate([record["corners"].reshape(-1, 2) for record in records])
    corner_bounds = [float(np.min(all_corners[:, 0]) / size[0]),
                     float(np.min(all_corners[:, 1]) / size[1]),
                     float(np.max(all_corners[:, 0]) / size[0]),
                     float(np.max(all_corners[:, 1]) / size[1])]
    tilts = []
    distances = []
    board_center = np.mean(grid, axis=0).reshape(3, 1)
    for rvec, tvec in zip(rvecs, tvecs):
        R, _ = cv2.Rodrigues(rvec)
        tilts.append(float(np.degrees(np.arccos(np.clip(abs(R[2, 2]), 0, 1)))))
        distances.append(float(np.linalg.norm(R @ board_center + tvec)))
    cell_counts = np.zeros((3, 3), dtype=int)
    for center_x, center_y in centers:
        cell_counts[min(2, int(3 * center_y / size[1])), min(2, int(3 * center_x / size[0]))] += 1
    warnings = []
    if image_source in ("live_preview", "mvs_gui"):
        if any(item.get("settings") != metadata[0].get("settings") for item in metadata):
            warnings.append("Camera settings varied across preview images")
        if any(item.get("frame", {}).get("lost_packets", 0) for item in metadata):
            warnings.append("Some preview images report lost packets")
    if len(records) < 25:
        warnings.append("Fewer than 25 valid views; collect more varied images for a stronger estimate")
    if np.count_nonzero(cell_counts) < 5:
        warnings.append("Board centers cover fewer than five of nine image regions")
    if corner_bounds[0] > .2 or corner_bounds[1] > .2 or corner_bounds[2] < .8 or corner_bounds[3] < .8:
        warnings.append("Detected corners do not reach near all four sides of the image")
    if max(tilts) < 15:
        warnings.append("No strongly tilted board view; add left/right and up/down tilts")
    if max(distances) / min(distances) < 1.2:
        warnings.append("Limited range of board distances; add near and far views without changing focus")
    if np.max([record["board_area_fraction"] for record in records]) < .08:
        warnings.append("Board is small in every image; move closer or use a larger board at fixed focus")
    result = {
        "status": "estimated_from_images",
        "image_source": image_source,
        "camera_model": manifest.get("camera_model", "unknown"),
        "camera_serial": serial, "lens_expected": "MVL-KF0814M-12MPE",
        "image_width": size[0], "image_height": size[1],
        "model": ("OpenCV pinhole, k1 k2 p1 p2 fitted, k3 fixed to zero, plumb_bob"
                  if args.fix_k3 else "OpenCV pinhole, 5 coefficient radial/tangential plumb_bob"),
        "calibration_flags": int(cv2.CALIB_FIX_K3 if args.fix_k3 else 0),
        "K": K.tolist(), "D_order": ["k1", "k2", "p1", "p2", "k3"], "D": D.tolist(),
        "intrinsic_standard_deviation_order": ["fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3"],
        "intrinsic_standard_deviation": std_i[:9].tolist(),
        "calibration_rms_px": rms, "square_mm": args.square_mm,
        "board_inner_corners": list(PATTERN), "total_images": total, "used_images": len(records),
        "failed_images": failures, "camera_settings": manifest.get("settings"),
        "board_center_region_counts": cell_counts.tolist(),
        "detected_corner_bounds_normalized_xyxy": corner_bounds,
        "board_tilt_degrees": tilts, "board_center_distance_mm": distances,
        "warnings": warnings,
        "images": [{key: value for key, value in record.items() if key not in ("corners", "path")} for record in records],
        "validation": validation,
        "opencv_version": cv2.__version__,
    }
    (output / "intrinsics.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    model_name = manifest.get("camera_model", "camera").lower().replace("-", "_")
    (output / "camera_info.yaml").write_text(ros_yaml(f"{model_name}_{serial}", size, K, D), encoding="utf-8")
    print(f"Used {len(records)}/{total} images; RMS = {rms:.4f} px")
    print("K =\n", K)
    print("D [k1,k2,p1,p2,k3] =", D)
    if validation:
        values = [item["reprojection_rms_px"] for item in validation["per_image"] if "reprojection_rms_px" in item]
        print(f"Separate validation: {len(values)} images, mean RMS = {np.mean(values):.4f} px" if values else "No valid validation images")
    for warning in warnings:
        print("WARNING:", warning)
    print("Saved", output / "intrinsics.json", "and", output / "camera_info.yaml")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, cv2.error, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
