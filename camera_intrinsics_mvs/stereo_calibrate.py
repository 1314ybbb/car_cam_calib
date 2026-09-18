#!/usr/bin/env python3
"""Estimate camera 1 -> camera 2 extrinsics from paired 9x6 chessboard images."""

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np

from calibrate import PATTERN, object_grid


def load_intrinsics(path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if data.get("board_inner_corners") != list(PATTERN) or data.get("D_order") != ["k1", "k2", "p1", "p2", "k3"]:
        raise RuntimeError(f"内参规格不支持：{path}")
    K = np.asarray(data["K"], dtype=np.float64)
    D = np.asarray(data["D"], dtype=np.float64).reshape(-1)
    if K.shape != (3, 3) or D.size != 5 or not np.isfinite(K).all() or not np.isfinite(D).all():
        raise RuntimeError(f"内参 K/D 无效：{path}")
    if not data.get("camera_serial") or not data.get("camera_model"):
        raise RuntimeError(f"内参缺少相机身份：{path}")
    return data, K, D


def check_capture_settings(metadata, intrinsics, path):
    if (metadata.get("camera_model"), metadata.get("camera_serial")) != (
            intrinsics["camera_model"], intrinsics["camera_serial"]):
        raise RuntimeError(f"{path.name} 与所选内参的相机身份不符")
    frame = metadata.get("frame", {})
    if frame.get("width") and frame.get("height"):
        size = (int(frame["width"]), int(frame["height"]))
        if size != (int(intrinsics["image_width"]), int(intrinsics["image_height"])):
            raise RuntimeError(f"{path.name} 与所选内参的分辨率不符")
    current = metadata.get("settings") or {}
    original = intrinsics.get("camera_settings") or {}
    for key in ("OffsetX", "OffsetY", "BinningHorizontal", "BinningVertical",
                "DecimationHorizontal", "DecimationVertical"):
        if key in current and key in original and int(current[key]) != int(original[key]):
            raise RuntimeError(f"{path.name} 的 {key} 与内参标定时不一致")


def paired_images(folder, intrinsics):
    folder = Path(folder).expanduser().resolve()
    if not folder.is_dir():
        raise RuntimeError(f"图片目录不存在：{folder}")
    pairs = {}
    for metadata_path in sorted(folder.glob("frame_*.json")):
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        pair_id = metadata.get("pair_id")
        if pair_id is None:
            continue
        if isinstance(pair_id, bool) or not isinstance(pair_id, int) or pair_id < 1:
            raise RuntimeError(f"{metadata_path.name} 的 pair_id 必须为正整数")
        check_capture_settings(metadata, intrinsics, metadata_path)
        if pair_id in pairs:
            raise RuntimeError(f"{folder} 中外参组号 {pair_id} 重复")
        image_path = metadata_path.with_suffix(".png")
        if not image_path.is_file():
            raise RuntimeError(f"缺少与 {metadata_path.name} 同名的 PNG")
        pairs[pair_id] = image_path
    return pairs


def detect_pairs(images1, images2, size1, size2):
    records, skipped = [], []
    for pair_id in sorted(set(images1) | set(images2)):
        if pair_id not in images1 or pair_id not in images2:
            skipped.append({"pair_id": pair_id, "reason": "另一台相机缺少此组照片"})
            continue
        captures = []
        reason = None
        for image_path, size in ((images1[pair_id], size1), (images2[pair_id], size2)):
            image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if image is None:
                reason = f"无法读取 {image_path.name}"
                break
            if (image.shape[1], image.shape[0]) != size:
                reason = f"{image_path.name} 分辨率与内参不一致"
                break
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCornersSB(gray, PATTERN)
            if not found:
                reason = f"{image_path.name} 未检出完整 9×6 棋盘"
                break
            captures.append((image_path, image, corners.astype(np.float32)))
        if reason:
            skipped.append({"pair_id": pair_id, "reason": reason})
        else:
            records.append({"pair_id": pair_id, "camera1": captures[0], "camera2": captures[1]})
    return records, skipped


def reprojection_rms(grid, corners, rvec, tvec, K, D):
    projected, _ = cv2.projectPoints(grid, rvec, tvec, K, D)
    residual = projected.reshape(-1, 2) - corners.reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))


def orient_pair_corners(records, K1, D1, K2, D2, grid):
    """Resolve the 180-degree corner-order ambiguity of a plain chessboard."""
    candidates = []
    for record in records:
        ok1, rvec1, tvec1 = cv2.solvePnP(grid, record["camera1"][2], K1, D1)
        if not ok1:
            raise RuntimeError(f"外参组号 {record['pair_id']} 的相机 1 位姿计算失败")
        R1, _ = cv2.Rodrigues(rvec1)
        options = []
        for flip in (False, True):
            corners = record["camera2"][2][::-1].copy() if flip else record["camera2"][2]
            ok2, rvec2, tvec2 = cv2.solvePnP(grid, corners, K2, D2)
            if not ok2:
                raise RuntimeError(f"外参组号 {record['pair_id']} 的相机 2 位姿计算失败")
            R2, _ = cv2.Rodrigues(rvec2)
            R = R2 @ R1.T
            T = tvec2 - R @ tvec1
            options.append((R, T, flip))
        candidates.append(options)

    def distance(a, b):
        delta = a[0] @ b[0].T
        angle = np.degrees(np.arccos(np.clip((np.trace(delta) - 1) / 2, -1, 1)))
        translation = np.linalg.norm(a[1] - b[1])
        return float((angle / 3) ** 2 + (translation / 100) ** 2)

    seeds = [item for options in candidates for item in options]
    best = min(seeds, key=lambda seed: sum(min(min(distance(option, seed) for option in options), 100)
                                            for options in candidates))
    flipped = []
    for record, options in zip(records, candidates):
        chosen = min(options, key=lambda option: distance(option, best))
        if chosen[2]:
            path, image, corners = record["camera2"]
            record["camera2"] = (path, image, corners[::-1].copy())
            flipped.append(record["pair_id"])
    return flipped


def fit_stereo(records, K1, D1, K2, D2, square_mm, size1, size2):
    grid = object_grid(square_mm)
    flipped = orient_pair_corners(records, K1, D1, K2, D2, grid)
    objects = [grid.copy() for _ in records]
    points1 = [record["camera1"][2] for record in records]
    points2 = [record["camera2"][2] for record in records]
    rms, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
        objects, points1, points2, K1.copy(), D1.copy(), K2.copy(), D2.copy(), size1,
        flags=cv2.CALIB_FIX_INTRINSIC,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-9))
    if not np.isfinite(R).all() or not np.isfinite(T).all():
        raise RuntimeError("外参求解得到了非有限数值")
    baseline = float(np.linalg.norm(T))
    if baseline < 1e-6:
        raise RuntimeError("相机基线接近零；检查是否误将同一相机照片配成两台相机")
    per_pair = []
    for record in records:
        corners1, corners2 = record["camera1"][2], record["camera2"][2]
        ok1, rvec1, tvec1 = cv2.solvePnP(grid, corners1, K1, D1)
        ok2, rvec2, tvec2 = cv2.solvePnP(grid, corners2, K2, D2)
        if not ok1 or not ok2:
            raise RuntimeError(f"外参组号 {record['pair_id']} 的单相机位姿计算失败")
        rotation1, _ = cv2.Rodrigues(rvec1)
        rotation2, _ = cv2.Rodrigues(rvec2)
        pair_rotation = rotation2 @ rotation1.T
        pair_translation = tvec2 - pair_rotation @ tvec1
        delta_rotation = pair_rotation @ R.T
        angle = float(np.degrees(np.arccos(np.clip((np.trace(delta_rotation) - 1) / 2, -1, 1))))
        per_pair.append({
            "pair_id": record["pair_id"],
            "camera1_image": record["camera1"][0].name,
            "camera2_image": record["camera2"][0].name,
            "camera1_reprojection_rms_px": reprojection_rms(grid, corners1, rvec1, tvec1, K1, D1),
            "camera2_reprojection_rms_px": reprojection_rms(grid, corners2, rvec2, tvec2, K2, D2),
            "relative_rotation_difference_deg": angle,
            "relative_translation_difference_mm": float(np.linalg.norm(pair_translation - T)),
        })
    yaw = float(np.degrees(np.arctan2(R[0, 2], R[2, 2])))
    rotation_angle = float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))
    return {
        "method": "OpenCV stereoCalibrate, CALIB_FIX_INTRINSIC, 9x6 chessboard",
        "camera1_image_size": list(size1), "camera2_image_size": list(size2), "square_mm": square_mm,
        "stereo_rms_px": float(rms), "used_pairs": len(records),
        "pair_ids": [record["pair_id"] for record in records],
        "camera2_corner_order_reversed_pairs": flipped,
        "R_camera1_to_camera2": R.tolist(),
        "T_camera1_to_camera2_mm": T.reshape(3).tolist(),
        "camera2_center_in_camera1_mm": (-R.T @ T).reshape(3).tolist(),
        "baseline_mm": baseline,
        "rotation_angle_deg": rotation_angle,
        "yaw_about_camera_y_deg": yaw,
        "E": E.tolist(), "F": F.tolist(), "per_pair": per_pair,
    }


def save_annotated(records, output):
    annotations = output / "detected_corners"
    annotations.mkdir()
    for record in records:
        for camera_name in ("camera1", "camera2"):
            _, image, corners = record[camera_name]
            shown = image.copy()
            cv2.drawChessboardCorners(shown, PATTERN, corners, True)
            scale = min(1.0, 1200 / max(shown.shape[1], shown.shape[0]))
            if scale < 1:
                shown = cv2.resize(shown, None, fx=scale, fy=scale)
            cv2.imwrite(str(annotations / f"pair_{record['pair_id']:04d}_{camera_name}.jpg"), shown,
                        [cv2.IMWRITE_JPEG_QUALITY, 85])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera1-images", required=True, type=Path)
    parser.add_argument("--camera2-images", required=True, type=Path)
    parser.add_argument("--intrinsics1", required=True, type=Path)
    parser.add_argument("--intrinsics2", required=True, type=Path)
    parser.add_argument("--square-mm", required=True, type=float)
    parser.add_argument("--output", required=True, type=Path, help="new result directory")
    parser.add_argument("--expected-angle-deg", type=float, default=60.0)
    args = parser.parse_args()
    if args.square_mm <= 0:
        parser.error("--square-mm must be positive")
    folder1 = args.camera1_images.expanduser().resolve()
    folder2 = args.camera2_images.expanduser().resolve()
    if folder1 == folder2:
        raise RuntimeError("两台相机必须使用不同的图片文件夹")
    intrinsics1, K1, D1 = load_intrinsics(args.intrinsics1)
    intrinsics2, K2, D2 = load_intrinsics(args.intrinsics2)
    if intrinsics1["camera_serial"] == intrinsics2["camera_serial"]:
        raise RuntimeError("两份内参属于同一台相机")
    size1 = (int(intrinsics1["image_width"]), int(intrinsics1["image_height"]))
    size2 = (int(intrinsics2["image_width"]), int(intrinsics2["image_height"]))
    images1 = paired_images(folder1, intrinsics1)
    images2 = paired_images(folder2, intrinsics2)
    if not images1 or not images2:
        raise RuntimeError("两台相机的文件夹都需要通过上位机的外参配对模式保存照片")
    records, skipped = detect_pairs(images1, images2, size1, size2)
    print(f"找到 {len(set(images1) & set(images2))} 组配对照片；完整检出棋盘 {len(records)} 组", flush=True)
    if len(records) < 5:
        raise RuntimeError(f"只有 {len(records)} 组有效照片；至少需要 5 组，建议 10～20 组不同棋盘姿态")
    result = fit_stereo(records, K1, D1, K2, D2, args.square_mm, size1, size2)
    result["camera1"] = {"model": intrinsics1["camera_model"], "serial": intrinsics1["camera_serial"],
                         "images": str(folder1), "intrinsics": str(args.intrinsics1.expanduser().resolve())}
    result["camera2"] = {"model": intrinsics2["camera_model"], "serial": intrinsics2["camera_serial"],
                         "images": str(folder2), "intrinsics": str(args.intrinsics2.expanduser().resolve())}
    result["skipped_pairs"] = skipped
    result["expected_angle_deg"] = args.expected_angle_deg
    warnings = []
    if len(records) < 10:
        warnings.append("有效配对少于 10 组；建议增加不同棋盘姿态")
    if result["stereo_rms_px"] > 1.0:
        warnings.append("双目重投影误差大于 1 px；检查图像同步、棋盘是否移动及角点质量")
    if abs(abs(result["yaw_about_camera_y_deg"]) - args.expected_angle_deg) > 5:
        warnings.append("估计的水平偏航角与预期夹角相差超过 5°；检查安装方向及拍摄配对")
    if max(item["relative_rotation_difference_deg"] for item in result["per_pair"]) > 3:
        warnings.append("部分配对的单张位姿与整体结果相差超过 3°；请检查配对和角点可视化")
    result["warnings"] = warnings
    output = args.output.expanduser().resolve()
    if output.exists():
        raise RuntimeError(f"结果目录已存在：{output}")
    output.mkdir(parents=True)
    save_annotated(records, output)
    (output / "extrinsics.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"双目 RMS {result['stereo_rms_px']:.4f} px；基线 {result['baseline_mm']:.1f} mm；"
          f"偏航角 {result['yaw_about_camera_y_deg']:.2f}°", flush=True)
    for warning in warnings:
        print(f"WARNING: {warning}", flush=True)
    for rejected in skipped:
        print(f"跳过组 {rejected['pair_id']}: {rejected['reason']}", flush=True)
    print(f"已保存 {output / 'extrinsics.json'}", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, OSError, ValueError, KeyError, cv2.error) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
