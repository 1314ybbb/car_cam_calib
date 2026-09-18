"""Qt dialog for paired chessboard extrinsic calibration."""

import json
from pathlib import Path
import sys
import time

from PyQt5.QtCore import QProcess, Qt
from PyQt5.QtWidgets import (QDialog, QDoubleSpinBox, QFileDialog, QGridLayout,
                             QHBoxLayout, QLabel, QLineEdit, QPlainTextEdit,
                             QPushButton, QVBoxLayout)


class StereoCalibrationDialog(QDialog):
    def __init__(self, camera_window):
        super().__init__(camera_window)
        self.camera_window = camera_window
        self.process = None
        self.output = None
        self.setWindowTitle("双相机棋盘格外参标定")
        self.resize(900, 500)
        layout = QVBoxLayout(self)
        hint = QLabel("每台相机选择一个照片文件夹，按组号配对：同一组内先固定棋盘，再由两台相机各拍一张；"
                      "拍完这一组才能移动棋盘并切换组号。组与组之间可以改变棋盘位姿。"
                      "手持或运动中的棋盘需要双相机同步触发，手动填写相同组号不能保证配对有效。建议 10～20 组。")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        grid = QGridLayout()
        self.folders = []
        self.intrinsics = []
        for index in range(2):
            row = index * 2
            grid.addWidget(QLabel(f"相机 {index + 1} 图片文件夹"), row, 0)
            folder = QLineEdit()
            folder.setReadOnly(True)
            self.folders.append(folder)
            grid.addWidget(folder, row, 1)
            choose_folder = QPushButton("选择文件夹")
            choose_folder.clicked.connect(lambda _checked=False, i=index: self.choose_folder(i))
            grid.addWidget(choose_folder, row, 2)
            use_current = QPushButton("填入当前相机")
            use_current.clicked.connect(lambda _checked=False, i=index: self.use_current(i))
            grid.addWidget(use_current, row, 3)

            grid.addWidget(QLabel(f"相机 {index + 1} 内参"), row + 1, 0)
            intrinsics = QLineEdit()
            intrinsics.setReadOnly(True)
            self.intrinsics.append(intrinsics)
            grid.addWidget(intrinsics, row + 1, 1)
            choose_intrinsics = QPushButton("选择 intrinsics.json")
            choose_intrinsics.clicked.connect(lambda _checked=False, i=index: self.choose_intrinsics(i))
            grid.addWidget(choose_intrinsics, row + 1, 2)
        grid.setColumnStretch(1, 1)
        layout.addLayout(grid)

        options = QHBoxLayout()
        options.addWidget(QLabel("实测方格边长 (mm)"))
        self.square_spin = QDoubleSpinBox()
        self.square_spin.setRange(0.01, 1000)
        self.square_spin.setDecimals(3)
        self.square_spin.setValue(camera_window.square_spin.value())
        options.addWidget(self.square_spin)
        options.addWidget(QLabel("预期水平夹角 (°)"))
        self.angle_spin = QDoubleSpinBox()
        self.angle_spin.setRange(0, 180)
        self.angle_spin.setDecimals(1)
        self.angle_spin.setValue(60)
        options.addWidget(self.angle_spin)
        self.run_button = QPushButton("计算双相机外参")
        self.run_button.clicked.connect(self.start)
        options.addWidget(self.run_button)
        layout.addLayout(options)

        self.result_label = QLabel("外参结果：尚未计算")
        self.result_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.result_label.setWordWrap(True)
        layout.addWidget(self.result_label)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        layout.addWidget(self.log, 1)

    def choose_folder(self, index):
        current = self.folders[index].text()
        start = str(Path(current).parent if current else Path.home() / "桌面")
        name = QFileDialog.getExistingDirectory(
            self, f"选择相机 {index + 1} 的外参配对照片文件夹", start,
            options=QFileDialog.ShowDirsOnly | QFileDialog.DontUseNativeDialog)
        if name:
            self.folders[index].setText(name)

    def choose_intrinsics(self, index):
        start = self.intrinsics[index].text() or str(Path.home())
        name, _ = QFileDialog.getOpenFileName(self, f"选择相机 {index + 1} 的 intrinsics.json",
                                              start, "内参文件 (intrinsics.json);;JSON (*.json)")
        if name:
            self.intrinsics[index].setText(name)

    def use_current(self, index):
        window = self.camera_window
        if window.selected is None or window.output_dir is None:
            self.log.appendPlainText("请先在主界面连接相机并选择该相机的图片文件夹")
            return
        target = f"cam{index + 1}"
        selected = next((item for item in (window.selected, window.secondary_selected)
                         if item is not None and window.camera_position(item["serial"]) == target), None)
        if selected is None:
            self.log.appendPlainText(f"{target} 尚未连接；请手动选择它的照片目录和内参")
            return
        self.folders[index].setText(str(window.pair_directory(selected)))
        intrinsic_path = window.matching_intrinsics_path(selected)
        if intrinsic_path is not None:
            self.intrinsics[index].setText(str(intrinsic_path))
        else:
            self.log.appendPlainText(f"{target} 尚未找到匹配内参；请点击“选择 intrinsics.json”")

    def start(self):
        if self.process is not None and self.process.state() != QProcess.NotRunning:
            return
        paths = [field.text().strip() for field in self.folders + self.intrinsics]
        if any(not path for path in paths):
            self.log.appendPlainText("请填齐两台相机各自的图片文件夹和 intrinsics.json")
            return
        if not all(Path(path).exists() for path in paths):
            self.log.appendPlainText("某个图片文件夹或内参文件不存在")
            return
        if Path(paths[0]).resolve() == Path(paths[1]).resolve():
            self.log.appendPlainText("两台相机必须使用不同的图片文件夹")
            return
        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = Path(paths[0]).resolve() / f"stereo_{stamp}"
        self.output = base
        suffix = 2
        while self.output.exists():
            self.output = Path(f"{base}_{suffix}")
            suffix += 1
        arguments = [str(Path(__file__).with_name("stereo_calibrate.py")),
                     "--camera1-images", paths[0], "--camera2-images", paths[1],
                     "--intrinsics1", paths[2], "--intrinsics2", paths[3],
                     "--square-mm", str(self.square_spin.value()),
                     "--expected-angle-deg", str(self.angle_spin.value()),
                     "--output", str(self.output)]
        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self.read_output)
        self.process.finished.connect(self.calibration_finished)
        self.process.errorOccurred.connect(self.error)
        self.log.clear()
        self.log.appendPlainText(f"正在计算，结果目录：{self.output}")
        self.process.start(sys.executable, arguments)
        self.run_button.setEnabled(False)

    def read_output(self):
        if self.process is not None:
            output = bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="replace")
            self.log.appendPlainText(output.rstrip())

    def error(self, _error):
        self.log.appendPlainText(f"程序错误：{self.process.errorString()}")
        self.run_button.setEnabled(True)

    def calibration_finished(self, exit_code, _status):
        self.read_output()
        path = self.output / "extrinsics.json"
        if exit_code == 0 and path.is_file():
            data = json.loads(path.read_text(encoding="utf-8"))
            self.result_label.setText(
                f"外参：{path}\n有效 {data['used_pairs']} 组；RMS {data['stereo_rms_px']:.3f} px；"
                f"基线 {data['baseline_mm']:.1f} mm；偏航角 {data['yaw_about_camera_y_deg']:.2f}°")
        else:
            review = self.output / "pair_review.json"
            if review.is_file():
                self.result_label.setText(f"配对一致性检查未通过；未生成可用外参。检查 {review}")
            else:
                self.result_label.setText("外参计算失败；请查看下方日志")
        self.run_button.setEnabled(True)
