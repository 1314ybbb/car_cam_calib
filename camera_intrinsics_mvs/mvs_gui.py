#!/usr/bin/env python3
"""Hikrobot camera GUI for paired capture, calibration, and live board pose."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
from PyQt5.QtCore import QProcess, Qt, QTimer
from PyQt5.QtGui import QImage, QKeySequence, QPixmap
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDoubleSpinBox,
                             QFileDialog, QHBoxLayout, QInputDialog, QLabel, QLineEdit, QMainWindow,
                             QMessageBox, QPlainTextEdit, QPushButton, QSlider,
                             QShortcut, QSpinBox, QVBoxLayout, QWidget)

from calibrate import object_grid
from mvs_capture import (DEFAULT_MVS, camera_metadata, check, enumerate_cameras,
                         grab_frame, load_sdk, select_camera)
from stereo_gui import StereoCalibrationDialog


WINDOW_TITLE = "Hikrobot 相机标定上位机"
STEREO_PAIRS_SUBDIR = "stereo_pairs"


def next_frame_path(folder):
    """Find the next unused image/JSON pair without overwriting existing data."""
    numbers = []
    for path in folder.glob("frame_*.png"):
        suffix = path.stem.removeprefix("frame_")
        if suffix.isdigit():
            numbers.append(int(suffix))
    index = max(numbers, default=0) + 1
    while True:
        image = folder / f"frame_{index:04d}.png"
        if not image.exists() and not image.with_suffix(".json").exists():
            return image
        index += 1


def validate_capture_folder(folder, model, serial):
    """Keep images from different physical cameras in separate directories."""
    for path in folder.glob("frame_*.json"):
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("camera_model") != model or data.get("camera_serial") != serial:
            raise RuntimeError(f"{path.name} 属于其他相机；请为 {serial} 选择单独文件夹")


def calibration_images(folder, selected=None):
    """Check the image/JSON pairing produced by this GUI before fitting."""
    images = sorted(path for path in folder.iterdir() if path.suffix.lower() in
                    {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"})
    identities = set()
    dimensions = set()
    intrinsic_images = []
    for image in images:
        metadata_path = image.with_suffix(".json")
        if not metadata_path.is_file():
            raise RuntimeError(f"{image.name} 缺少同名 JSON")
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        if metadata.get("pair_id") is not None:
            continue
        intrinsic_images.append(image)
        model, serial = metadata.get("camera_model"), metadata.get("camera_serial")
        if not model or not serial:
            raise RuntimeError(f"{metadata_path.name} 缺少相机型号或序列号")
        identities.add((model, serial))
        frame = metadata.get("frame", {})
        if frame.get("width") and frame.get("height"):
            dimensions.add((int(frame["width"]), int(frame["height"])))
    if len(identities) > 1:
        raise RuntimeError("文件夹混有不同相机的图像；请按相机分开保存")
    if selected and identities and identities != {(selected["model"], selected["serial"])}:
        raise RuntimeError("图片不属于当前连接的相机；请选择对应相机或断开后离线标定")
    if len(dimensions) > 1:
        raise RuntimeError("文件夹混有不同分辨率的图像；请分开标定")
    return intrinsic_images


def read_intrinsics(path, model=None, serial=None, size=None):
    data = json.loads(path.read_text(encoding="utf-8"))
    K = np.asarray(data["K"], dtype=np.float64)
    D = np.asarray(data["D"], dtype=np.float64).reshape(-1)
    if K.shape != (3, 3) or D.size < 4 or not np.isfinite(K).all() or not np.isfinite(D).all():
        raise RuntimeError("内参文件中的 K 或 D 无效")
    if data.get("D_order") != ["k1", "k2", "p1", "p2", "k3"] or D.size != 5:
        raise RuntimeError("内参畸变模型不支持；需要本工程的五系数针孔模型")
    if data.get("board_inner_corners") != [9, 6]:
        raise RuntimeError("内参对应的棋盘规格不是 9×6 内角点")
    if model is not None and data.get("camera_model") != model:
        raise RuntimeError(f"内参型号 {data.get('camera_model')} 与当前相机 {model} 不符")
    if serial is not None and data.get("camera_serial") != serial:
        raise RuntimeError(f"内参序列号 {data.get('camera_serial')} 与当前相机 {serial} 不符")
    expected_size = (int(data["image_width"]), int(data["image_height"]))
    if size is not None and expected_size != size:
        raise RuntimeError(f"内参分辨率 {expected_size} 与当前图像 {size} 不符")
    square_mm = float(data["square_mm"])
    if square_mm <= 0:
        raise RuntimeError("内参文件中的方格尺寸无效")
    return data, K, D, square_mm


def geometry_matches(data, settings):
    original = data.get("camera_settings") or {}
    return all(int(original[key]) == int(settings[key]) for key in
               ("OffsetX", "OffsetY", "BinningHorizontal", "BinningVertical",
                "DecimationHorizontal", "DecimationVertical")
               if key in original and key in settings)


def find_intrinsics_files(selected, settings=None, output_dir=None):
    """Find this camera's saved intrinsics in project and desktop result folders."""
    serial = selected["serial"]
    roots = [Path(__file__).resolve().parent.parent / "results" / serial,
             Path.home() / "codex_prj/car_cam_calib/results" / serial,
             Path.home() / "桌面/相机内参标定" / serial]
    paths = set()
    for root in roots:
        if root.is_dir():
            paths.update(path.resolve() for path in root.rglob("intrinsics.json"))
    if output_dir is not None:
        paths.update(path.resolve() for path in output_dir.glob("calibration_*/intrinsics.json"))
    candidates = []
    size = (int(settings["Width"]), int(settings["Height"])) if settings else None
    for path in paths:
        try:
            data, _, _, _ = read_intrinsics(path, selected["model"], serial, size)
            if settings and not geometry_matches(data, settings):
                continue
            candidates.append((path, data))
        except (RuntimeError, OSError, ValueError, KeyError, json.JSONDecodeError):
            continue
    return sorted(candidates, key=lambda item: (
        0 if "calibration_recommended" in str(item[0]) else 1, -item[0].stat().st_mtime))


def estimate_board_pose(image, K, D, square_mm):
    """Return detected corners, pose, board-center XYZ/range, and RMS."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCornersSB(gray, (9, 6))
    if not found:
        return None
    grid = object_grid(square_mm)
    success, rvec, tvec = cv2.solvePnP(grid, corners, K, D)
    if not success:
        return None
    R, _ = cv2.Rodrigues(rvec)
    center = (R @ np.array([[4 * square_mm], [2.5 * square_mm], [0.0]]) + tvec).reshape(3)
    projected, _ = cv2.projectPoints(grid, rvec, tvec, K, D)
    residual = projected.reshape(-1, 2) - corners.reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))
    return {"corners": corners, "rvec": rvec, "tvec": tvec,
            "center_mm": center, "range_mm": float(np.linalg.norm(center)), "rms_px": rms}


class CameraWindow(QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.sdk = None
        self.camera = None
        self.cameras = []
        self.selected = None
        self.trigger_original = None
        self.current_frame = None
        self.current_frame_info = None
        self.output_dir = None
        self.intrinsics_path = None
        self.intrinsics = None
        self.calibration_process = None
        self.calibration_output = None
        self.stereo_dialog = None
        self.pose_executor = ThreadPoolExecutor(max_workers=1)
        self.pose_future = None
        self.pose_visualization = None
        self.setting_gain_slider = False
        self.gain_min = 0.0
        self.gain_max = 1.0
        self.last_gain_requested = None
        self.timer = QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.update_frame)

        self.setWindowTitle(WINDOW_TITLE)
        self.resize(1200, 900)
        root = QWidget(self)
        layout = QVBoxLayout(root)

        camera_row = QHBoxLayout()
        camera_row.addWidget(QLabel("相机"))
        self.camera_combo = QComboBox()
        camera_row.addWidget(self.camera_combo, 1)
        self.refresh_button = QPushButton("刷新设备")
        self.refresh_button.clicked.connect(self.refresh_cameras)
        camera_row.addWidget(self.refresh_button)
        self.connect_button = QPushButton("连接相机")
        self.connect_button.clicked.connect(self.toggle_camera)
        camera_row.addWidget(self.connect_button)
        layout.addLayout(camera_row)

        folder_row = QHBoxLayout()
        self.folder_label = QLabel("图片目录：未选择")
        self.folder_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        folder_row.addWidget(self.folder_label, 1)
        self.folder_button = QPushButton("选择图片文件夹")
        self.folder_button.clicked.connect(self.choose_folder)
        folder_row.addWidget(self.folder_button)
        self.save_button = QPushButton("保存图片 + JSON")
        self.save_button.clicked.connect(self.save_frame)
        folder_row.addWidget(self.save_button)
        QShortcut(QKeySequence(Qt.Key_Space), self, activated=self.save_frame)
        layout.addLayout(folder_row)

        pair_row = QHBoxLayout()
        self.pair_toggle = QCheckBox("外参配对采集")
        pair_row.addWidget(self.pair_toggle)
        pair_row.addWidget(QLabel("组号"))
        self.pair_spin = QSpinBox()
        self.pair_spin.setRange(1, 999999)
        self.pair_spin.setEnabled(False)
        self.pair_toggle.toggled.connect(self.pair_spin.setEnabled)
        pair_row.addWidget(self.pair_spin)
        next_pair = QPushButton("下一组")
        next_pair.clicked.connect(lambda: self.pair_spin.setValue(self.pair_spin.value() + 1))
        pair_row.addWidget(next_pair)
        pair_hint = QLabel("每组固定棋盘，各拍 1 张；照片存入 stereo_pairs/；两张拍完再点“下一组”")
        pair_row.addWidget(pair_hint, 1)
        self.stereo_button = QPushButton("双相机棋盘格外参…")
        self.stereo_button.clicked.connect(self.open_stereo_dialog)
        pair_row.addWidget(self.stereo_button)
        layout.addLayout(pair_row)

        calibration_row = QHBoxLayout()
        calibration_row.addWidget(QLabel("棋盘格距 (mm)"))
        self.square_spin = QDoubleSpinBox()
        self.square_spin.setRange(0.01, 1000.0)
        self.square_spin.setDecimals(3)
        self.square_spin.setValue(args.square_mm)
        calibration_row.addWidget(self.square_spin)
        self.calibrate_button = QPushButton("一键生成内参")
        self.calibrate_button.clicked.connect(self.start_calibration)
        calibration_row.addWidget(self.calibrate_button)
        calibration_row.addWidget(QLabel("内参文件"))
        self.intrinsics_label = QLineEdit()
        self.intrinsics_label.setReadOnly(True)
        self.intrinsics_label.setPlaceholderText("连接相机后自动查找 intrinsics.json")
        calibration_row.addWidget(self.intrinsics_label, 1)
        self.find_intrinsics_button = QPushButton("查找已有内参")
        self.find_intrinsics_button.clicked.connect(self.choose_found_intrinsics)
        calibration_row.addWidget(self.find_intrinsics_button)
        self.intrinsics_button = QPushButton("浏览内参文件…")
        self.intrinsics_button.clicked.connect(self.choose_intrinsics)
        calibration_row.addWidget(self.intrinsics_button)
        self.pose_toggle = QCheckBox("实时位姿")
        self.pose_toggle.setEnabled(False)
        self.pose_toggle.toggled.connect(self.on_pose_toggled)
        calibration_row.addWidget(self.pose_toggle)
        layout.addLayout(calibration_row)

        gain_row = QHBoxLayout()
        gain_row.addWidget(QLabel("增益"))
        self.gain_slider = QSlider(Qt.Horizontal)
        self.gain_slider.setEnabled(False)
        self.gain_slider.valueChanged.connect(self.set_gain)
        gain_row.addWidget(self.gain_slider, 1)
        gain_row.addWidget(QLabel("输入"))
        self.gain_spin = QDoubleSpinBox()
        self.gain_spin.setDecimals(2)
        self.gain_spin.setSingleStep(0.1)
        self.gain_spin.setKeyboardTracking(False)
        self.gain_spin.setEnabled(False)
        self.gain_spin.valueChanged.connect(self.set_gain_value)
        gain_row.addWidget(self.gain_spin)
        self.apply_gain_button = QPushButton("应用增益")
        self.apply_gain_button.setEnabled(False)
        self.apply_gain_button.clicked.connect(self.apply_gain_input)
        gain_row.addWidget(self.apply_gain_button)
        self.gain_label = QLabel("未连接")
        self.gain_label.setToolTip("相机 SDK 读取的实际增益；设备可能只支持离散档位")
        gain_row.addWidget(self.gain_label)
        layout.addLayout(gain_row)

        self.video_label = QLabel("等待相机画面")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(960, 540)
        self.video_label.setStyleSheet("background: #17191c; color: #ddd")
        layout.addWidget(self.video_label, 1)
        self.pose_label = QLabel("位姿显示已关闭")
        layout.addWidget(self.pose_label)
        self.status_label = QLabel("就绪")
        layout.addWidget(self.status_label)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(110)
        layout.addWidget(self.log)
        self.setCentralWidget(root)

        if args.output_dir:
            self.set_output_dir(args.output_dir)
        if args.intrinsics:
            self.set_intrinsics(args.intrinsics)
        try:
            self.sdk = load_sdk(args.mvs_root)
            check(self.sdk.MvCamera.MV_CC_Initialize(), "initialize MVS")
            self.refresh_cameras()
        except (RuntimeError, OSError) as exc:
            self.sdk = None
            self.set_status(f"相机 SDK 不可用：{exc}；仍可从现有照片计算内参")
        if args.serial and self.sdk:
            index = self.camera_combo.findData(args.serial)
            if index >= 0:
                self.camera_combo.setCurrentIndex(index)
                self.connect_camera()
        self.update_controls()

    def set_status(self, message):
        self.status_label.setText(message)
        self.log.appendPlainText(message)

    def update_controls(self):
        connected = self.camera is not None
        self.connect_button.setText("断开相机" if connected else "连接相机")
        self.connect_button.setEnabled(connected or self.camera_combo.count() > 0)
        self.refresh_button.setEnabled(not connected and self.sdk is not None)
        self.camera_combo.setEnabled(not connected)
        self.save_button.setEnabled(connected and self.output_dir is not None)
        self.gain_slider.setEnabled(connected)
        self.gain_spin.setEnabled(connected)
        self.apply_gain_button.setEnabled(connected)
        self.find_intrinsics_button.setEnabled(connected)
        self.pose_toggle.setEnabled(connected and self.intrinsics is not None)
        if not connected or self.intrinsics is None:
            self.pose_toggle.setChecked(False)
        busy = self.calibration_process is not None and self.calibration_process.state() != QProcess.NotRunning
        self.calibrate_button.setEnabled(self.output_dir is not None and not busy)

    def refresh_cameras(self):
        if self.sdk is None or self.camera is not None:
            return
        try:
            self.cameras = enumerate_cameras(self.sdk)
            self.camera_combo.clear()
            seen = set()
            for entry in self.cameras:
                if entry["serial"] in seen:
                    continue
                seen.add(entry["serial"])
                self.camera_combo.addItem(f"{entry['model']}  {entry['serial']}  {entry['ip']}", entry["serial"])
            self.set_status(f"发现 {len(seen)} 台相机")
        except (RuntimeError, OSError) as exc:
            self.set_status(f"枚举相机失败：{exc}")
        self.update_controls()

    def toggle_camera(self):
        if self.camera is None:
            self.connect_camera()
        else:
            self.disconnect_camera()

    def connect_camera(self):
        serial = self.camera_combo.currentData()
        if self.sdk is None or not serial:
            self.set_status("请先选择相机")
            return
        try:
            self.selected = select_camera(self.cameras, serial)
            if self.output_dir is not None:
                try:
                    validate_capture_folder(self.output_dir, self.selected["model"], serial)
                except (RuntimeError, ValueError) as exc:
                    self.output_dir = None
                    self.folder_label.setText("图片目录：请为当前相机重新选择")
                    self.set_status(f"原图片目录不能用于当前相机：{exc}")
            self.camera = self.sdk.MvCamera()
            check(self.camera.MV_CC_CreateHandle(self.selected["device_info"]), "create camera handle")
            check(self.camera.MV_CC_OpenDevice(self.sdk.MV_ACCESS_Exclusive, 0), "open camera")
            trigger = self.sdk.MVCC_ENUMVALUE()
            if self.camera.MV_CC_GetEnumValue("TriggerMode", trigger) == 0:
                self.trigger_original = int(trigger.nCurValue)
            check(self.camera.MV_CC_SetEnumValue("TriggerMode", self.sdk.MV_TRIGGER_MODE_OFF), "disable trigger")
            gain_auto = self.sdk.MVCC_ENUMVALUE()
            if self.camera.MV_CC_GetEnumValue("GainAuto", gain_auto) == 0 and gain_auto.nCurValue != 0:
                check(self.camera.MV_CC_SetEnumValue("GainAuto", 0), "disable automatic gain")
            gain = self.sdk.MVCC_FLOATVALUE()
            check(self.camera.MV_CC_GetFloatValue("Gain", gain), "read gain")
            self.gain_min, self.gain_max = float(gain.fMin), float(gain.fMax)
            self.last_gain_requested = None
            self.setting_gain_slider = True
            self.gain_slider.setRange(0, max(1, round((self.gain_max - self.gain_min) * 10)))
            self.gain_spin.setRange(self.gain_min, self.gain_max)
            self.setting_gain_slider = False
            self.update_gain_display(float(gain.fCurValue))
            settings = camera_metadata(self.camera, self.sdk)
            if self.intrinsics_path:
                try:
                    self.set_intrinsics(self.intrinsics_path, self.selected, settings)
                except (RuntimeError, KeyError, ValueError) as exc:
                    self.intrinsics = None
                    self.intrinsics_label.setText("当前相机尚未选择匹配内参")
                    self.set_status(f"当前相机不能使用已选内参：{exc}")
            if self.intrinsics is None:
                candidates = find_intrinsics_files(self.selected, settings, self.output_dir)
                recommended = next((path for path, _ in candidates if "calibration_recommended" in str(path)), None)
                chosen = recommended or (candidates[0][0] if len(candidates) == 1 else None)
                if chosen is not None:
                    self.set_intrinsics(chosen, self.selected, settings)
                    self.set_status(f"已自动载入此相机的内参：{chosen}")
                elif candidates:
                    self.set_status(f"找到 {len(candidates)} 份内参；点击“查找已有内参”选择")
            check(self.camera.MV_CC_StartGrabbing(), "start grabbing")
            self.timer.start()
            self.setWindowTitle(f"{WINDOW_TITLE} - {self.selected['model']} / {serial}")
            self.set_status(f"已连接 {self.selected['model']} / {serial}")
        except (RuntimeError, cv2.error, OSError) as exc:
            self.disconnect_camera()
            self.set_status(f"连接失败：{exc}")
        self.update_controls()

    def disconnect_camera(self):
        self.timer.stop()
        if self.camera is not None:
            try:
                self.camera.MV_CC_StopGrabbing()
                if self.trigger_original is not None:
                    self.camera.MV_CC_SetEnumValue("TriggerMode", self.trigger_original)
                self.camera.MV_CC_CloseDevice()
                self.camera.MV_CC_DestroyHandle()
            except Exception as exc:
                self.set_status(f"关闭相机时出错：{exc}")
        self.camera = None
        self.selected = None
        self.current_frame = None
        self.current_frame_info = None
        self.trigger_original = None
        self.video_label.setText("等待相机画面")
        self.pose_label.setText("位姿显示已关闭")
        self.pose_visualization = None
        self.pose_future = None
        self.last_gain_requested = None
        self.gain_label.setText("未连接")
        self.setWindowTitle(WINDOW_TITLE)
        self.update_controls()

    def update_gain_display(self, value, preserve_input=False):
        self.setting_gain_slider = True
        self.gain_slider.setValue(round((value - self.gain_min) * 10))
        if not preserve_input:
            self.gain_spin.setValue(value)
        self.gain_label.setText(f"相机实际 {value:.4f}")
        self.setting_gain_slider = False

    def set_gain(self, position):
        if self.setting_gain_slider or self.camera is None:
            return
        requested = min(self.gain_max, self.gain_min + position / 10.0)
        self.write_gain(requested)

    def set_gain_value(self, value):
        if not self.setting_gain_slider and self.camera is not None:
            self.write_gain(value, preserve_input=True)

    def apply_gain_input(self):
        if self.camera is None:
            return
        self.setting_gain_slider = True
        self.gain_spin.interpretText()
        self.setting_gain_slider = False
        requested = self.gain_spin.value()
        if self.last_gain_requested is None or abs(requested - self.last_gain_requested) > 1e-6:
            self.write_gain(requested, preserve_input=True)

    def write_gain(self, requested, preserve_input=False):
        result = self.camera.MV_CC_SetFloatValue("Gain", requested)
        if result:
            self.set_status(f"设置增益失败：0x{result:08x}")
            return
        self.last_gain_requested = requested
        gain = self.sdk.MVCC_FLOATVALUE()
        if self.camera.MV_CC_GetFloatValue("Gain", gain) == 0:
            actual = float(gain.fCurValue)
            self.update_gain_display(actual, preserve_input=preserve_input)
            if preserve_input and abs(actual - requested) >= 0.005:
                self.set_status(f"请求增益 {requested:.2f}，相机实际回读 {actual:.4f}；设备可能使用离散增益档位")
        else:
            self.set_status("增益已写入，但读取相机实际增益失败")

    def update_frame(self):
        if self.camera is None:
            return
        try:
            frame, info = grab_frame(self.camera, self.sdk, 100)
        except RuntimeError:
            return
        if info["lost_packets"]:
            return
        self.current_frame, self.current_frame_info = frame, info
        display = frame
        if self.pose_toggle.isChecked() and self.intrinsics is not None:
            if self.pose_future is not None and self.pose_future.done():
                try:
                    self.pose_visualization, message = self.pose_future.result()
                    self.pose_label.setText(message)
                except (RuntimeError, cv2.error, ValueError) as exc:
                    self.pose_label.setText(f"位姿计算失败：{exc}")
                self.pose_future = None
            if self.pose_future is None:
                _, K, D, square_mm = self.intrinsics
                self.pose_future = self.pose_executor.submit(
                    self.process_pose_frame, frame.copy(), K.copy(), D.copy(), square_mm)
            if self.pose_visualization is not None:
                display = self.pose_visualization
        rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
        height, width = rgb.shape[:2]
        qimage = QImage(rgb.data, width, height, rgb.strides[0], QImage.Format_RGB888).copy()
        self.video_label.setPixmap(QPixmap.fromImage(qimage).scaled(
            self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    @staticmethod
    def process_pose_frame(frame, K, D, square_mm):
        pose = estimate_board_pose(frame, K, D, square_mm)
        if pose is None:
            return frame, "当前画面未检出完整 9×6 棋盘"
        cv2.drawChessboardCorners(frame, (9, 6), pose["corners"], True)
        cv2.drawFrameAxes(frame, K, D, pose["rvec"], pose["tvec"], 2 * square_mm, 2)
        x, y, z = pose["center_mm"]
        message = (f"棋盘中心 X={x:.1f}  Y={y:.1f}  Z={z:.1f} mm  |  "
                   f"光心距离={pose['range_mm']:.1f} mm  |  重投影 RMS={pose['rms_px']:.3f} px")
        return frame, message

    def set_output_dir(self, path):
        folder = Path(path).expanduser().resolve()
        folder.mkdir(parents=True, exist_ok=True)
        self.output_dir = folder
        self.folder_label.setText(f"图片目录：{folder}")
        self.update_controls()

    def choose_folder(self):
        start = str(self.output_dir or Path.home())
        name = QFileDialog.getExistingDirectory(self, "选择图片保存与标定目录", start)
        if name:
            try:
                self.set_output_dir(name)
                self.set_status(f"已选择图片目录：{name}")
            except OSError as exc:
                self.set_status(f"文件夹不可用：{exc}")

    def save_frame(self):
        if self.output_dir is None or self.camera is None or self.current_frame is None:
            self.set_status("请先连接相机并选择图片文件夹")
            return
        image_path = None
        try:
            pair_id = self.pair_spin.value() if self.pair_toggle.isChecked() else None
            capture_dir = self.output_dir / STEREO_PAIRS_SUBDIR if pair_id is not None else self.output_dir
            if pair_id is not None:
                capture_dir.mkdir(exist_ok=True)
            validate_capture_folder(capture_dir, self.selected["model"], self.selected["serial"])
            if pair_id is not None:
                for path in capture_dir.glob("frame_*.json"):
                    if json.loads(path.read_text(encoding="utf-8")).get("pair_id") == pair_id:
                        raise RuntimeError(f"外参组号 {pair_id} 已在当前相机文件夹保存；请换组号或检查照片")
                gray = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2GRAY)
                if not cv2.findChessboardCornersSB(gray, (9, 6))[0]:
                    raise RuntimeError("未检出完整 9×6 棋盘，本组未保存")
            image_path = next_frame_path(capture_dir)
            if not cv2.imwrite(str(image_path), self.current_frame):
                raise RuntimeError(f"无法保存图像：{image_path}")
            gray = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2GRAY)
            metadata = {
                "capture_app": "mvs_gui",
                "camera_model": self.selected["model"], "camera_serial": self.selected["serial"],
                "camera_ip": self.selected["ip"],
                "host_interface_ip": self.selected["host_interface_ip"],
                "sdk_version": f"0x{self.sdk.MvCamera.MV_CC_GetSDKVersion():08x}",
                "settings": camera_metadata(self.camera, self.sdk), "frame": self.current_frame_info,
                "image_stats": {"mean": float(np.mean(gray)), "min": int(gray.min()),
                                "max": int(gray.max())},
                "conversion": "MVS RGB8 then OpenCV BGR8", "capture_unix_ns": time.time_ns(),
            }
            if pair_id is not None:
                metadata["pair_id"] = pair_id
            image_path.with_suffix(".json").write_text(
                json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8")
            label = f"外参组 {pair_id}" if pair_id is not None else "普通采集"
            self.set_status(f"已保存 {image_path} 和同名 JSON（{label}）")
        except (RuntimeError, OSError, ValueError, json.JSONDecodeError, cv2.error) as exc:
            if image_path is not None and not image_path.with_suffix(".json").exists():
                image_path.unlink(missing_ok=True)
            self.set_status(f"保存失败：{exc}")

    def start_calibration(self):
        if self.output_dir is None:
            self.set_status("请先选择含棋盘图像的文件夹")
            return
        if self.calibration_process is not None and self.calibration_process.state() != QProcess.NotRunning:
            return
        try:
            images = calibration_images(self.output_dir, self.selected)
        except (RuntimeError, OSError, ValueError, json.JSONDecodeError) as exc:
            self.set_status(f"标定输入不可用：{exc}")
            return
        if len(images) < 15:
            self.set_status(f"当前目录只有 {len(images)} 张图像；至少需要 15 张有效棋盘照片")
            return
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.calibration_output = self.output_dir / f"calibration_{stamp}"
        self.calibration_process = QProcess(self)
        self.calibration_process.setProcessChannelMode(QProcess.MergedChannels)
        self.calibration_process.readyReadStandardOutput.connect(self.read_calibration_output)
        self.calibration_process.finished.connect(self.calibration_finished)
        self.calibration_process.errorOccurred.connect(self.calibration_error)
        arguments = [str(Path(__file__).with_name("calibrate.py")),
                     "--images", str(self.output_dir), "--output", str(self.calibration_output),
                     "--square-mm", str(self.square_spin.value()), "--fix-k3"]
        self.calibration_process.start(sys.executable, arguments)
        self.set_status(f"正在计算内参，输入 {len(images)} 张图像；结果目录 {self.calibration_output}")
        self.update_controls()

    def calibration_error(self, _error):
        self.set_status(f"标定程序启动或运行失败：{self.calibration_process.errorString()}")
        self.update_controls()

    def read_calibration_output(self):
        if self.calibration_process is None:
            return
        output = bytes(self.calibration_process.readAllStandardOutput()).decode("utf-8", errors="replace")
        for line in output.splitlines():
            self.log.appendPlainText(line)

    def calibration_finished(self, exit_code, _status):
        self.read_calibration_output()
        path = self.calibration_output / "intrinsics.json"
        if exit_code == 0 and path.is_file():
            try:
                self.set_intrinsics(path, self.selected,
                                    camera_metadata(self.camera, self.sdk) if self.camera else None)
                data = self.intrinsics[0]
                self.set_status(f"内参已生成：有效 {data['used_images']} 张，RMS {data['calibration_rms_px']:.4f} px")
            except (RuntimeError, KeyError, ValueError, OSError) as exc:
                self.set_status(f"内参已生成，但未载入实时位姿：{exc}")
        else:
            self.set_status(f"标定失败，退出码 {exit_code}；请查看下方日志")
        self.update_controls()

    def set_intrinsics(self, path, selected=None, settings=None):
        path = Path(path).expanduser().resolve()
        model = selected["model"] if selected else None
        serial = selected["serial"] if selected else None
        size = ((int(settings["Width"]), int(settings["Height"])) if settings else None)
        intrinsics = read_intrinsics(path, model, serial, size)
        original_settings = intrinsics[0].get("camera_settings") or {}
        if settings:
            for key in ("OffsetX", "OffsetY", "BinningHorizontal", "BinningVertical",
                        "DecimationHorizontal", "DecimationVertical"):
                if key in original_settings and key in settings and int(original_settings[key]) != int(settings[key]):
                    raise RuntimeError(f"当前相机的 {key} 与内参标定时不一致")
        self.intrinsics = intrinsics
        self.intrinsics_path = path
        self.pose_visualization = None
        self.pose_future = None
        self.intrinsics_label.setText(str(path))
        self.intrinsics_label.setToolTip(
            f"{path}\n相机 {intrinsics[0]['camera_serial']} | "
            f"{intrinsics[0]['image_width']}×{intrinsics[0]['image_height']} | "
            f"RMS {intrinsics[0].get('calibration_rms_px', float('nan')):.4f} px")
        self.update_controls()

    def choose_found_intrinsics(self):
        if self.selected is None or self.camera is None:
            self.set_status("请先连接要使用内参的相机")
            return
        settings = camera_metadata(self.camera, self.sdk)
        candidates = find_intrinsics_files(self.selected, settings, self.output_dir)
        if not candidates:
            self.set_status("未找到此相机的内参；可点击“浏览内参文件…”手动选择 intrinsics.json")
            return
        labels = [f"{item['camera_serial']} | {item['image_width']}×{item['image_height']} | "
                  f"RMS {item.get('calibration_rms_px', float('nan')):.4f} px | {path}"
                  for path, item in candidates]
        choice, ok = QInputDialog.getItem(self, "选择已有内参", "请选择与当前镜头及对焦设置对应的结果：",
                                          labels, 0, False)
        if ok:
            path = candidates[labels.index(choice)][0]
            try:
                self.set_intrinsics(path, self.selected, settings)
                self.set_status(f"已载入内参：{path}")
            except (RuntimeError, KeyError, ValueError, OSError) as exc:
                self.set_status(f"内参不能使用：{exc}")

    def choose_intrinsics(self):
        start = str(self.intrinsics_path or self.output_dir or Path.home())
        name, _ = QFileDialog.getOpenFileName(self, "选择内参 JSON", start, "JSON (*.json)")
        if name:
            try:
                settings = camera_metadata(self.camera, self.sdk) if self.camera else None
                self.set_intrinsics(name, self.selected, settings)
                self.set_status(f"已载入内参：{name}")
            except (RuntimeError, KeyError, ValueError, OSError) as exc:
                self.set_status(f"内参不能使用：{exc}")

    def open_stereo_dialog(self):
        if self.stereo_dialog is None:
            self.stereo_dialog = StereoCalibrationDialog(self)
        if not self.stereo_dialog.folders[0].text() and self.selected is not None:
            self.stereo_dialog.use_current(0)
        self.stereo_dialog.show()
        self.stereo_dialog.raise_()
        self.stereo_dialog.activateWindow()

    def on_pose_toggled(self, enabled):
        if not enabled:
            self.pose_visualization = None
            self.pose_future = None
            self.pose_label.setText("位姿显示已关闭")
        else:
            self.pose_label.setText("正在检测完整 9×6 棋盘…")

    def closeEvent(self, event):
        running_intrinsics = self.calibration_process is not None and self.calibration_process.state() != QProcess.NotRunning
        running_stereo = (self.stereo_dialog is not None and self.stereo_dialog.process is not None
                          and self.stereo_dialog.process.state() != QProcess.NotRunning)
        if running_intrinsics or running_stereo:
            answer = QMessageBox.question(self, "标定仍在进行", "退出将中断正在进行的标定计算，确定退出吗？")
            if answer != QMessageBox.Yes:
                event.ignore()
                return
        if running_intrinsics:
            self.calibration_process.terminate()
            self.calibration_process.waitForFinished(2000)
        if running_stereo:
            self.stereo_dialog.process.terminate()
            self.stereo_dialog.process.waitForFinished(2000)
        self.disconnect_camera()
        self.pose_executor.shutdown(wait=False, cancel_futures=True)
        if self.sdk is not None:
            self.sdk.MvCamera.MV_CC_Finalize()
        event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", help="initial camera serial number")
    parser.add_argument("--output-dir", type=Path, help="initial image folder; can be changed in GUI")
    parser.add_argument("--intrinsics", type=Path, help="initial intrinsics JSON; can be changed in GUI")
    parser.add_argument("--square-mm", type=float, default=50.0)
    parser.add_argument("--mvs-root", type=Path, default=DEFAULT_MVS)
    args = parser.parse_args()
    if args.square_mm <= 0:
        parser.error("--square-mm must be positive")
    app = QApplication(sys.argv[:1])
    window = CameraWindow(args)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
