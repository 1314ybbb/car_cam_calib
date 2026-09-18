# Hikrobot 双相机标定工程

本仓库包含海康机器人 GigE 相机的取帧上位机、棋盘格内参与双相机外参算法、参数和棋盘 PDF。**队友接手请先读 [双相机交接手册](HANDOFF_TWO_CAMERAS.md)**：其中有相机序列号与物理位置绑定、上次工作 IP、网络与 SDK 配置、启动命令和故障排查。按用户要求，2026-09-18 的原始照片没有上传。

本次实采的两台相机都自报 **MV-CS023-10GC**，分辨率 1920×1200：`cam1 = DB2189859`，`cam2 = DB2189878`。原计划型号 **MV-CS050-10GC** 与实采型号不同，仓库中的参数不能直接用于计划采购的型号。本次用户提供的镜头型号为 MVL-KF0814M-12MPE；逐台镜头和最终安装状态需现场确认。

## 仓库内容

- `camera_intrinsics_mvs/`：上位机、MVS SDK 取帧、内参/外参求解及启动脚本。详细操作见 [脚本说明](camera_intrinsics_mvs/README.md)。
- `results/capture_manifest_20260918.json`：两台相机内外参采集的 IP、曝光、增益、帧尺寸、数量与配对时间差摘要。2026-09-18 的原图仅在采集电脑本地。
- `results/DB2189859/calibration_20260918_111338/`、`results/DB2189878/calibration_20260918_103029/`：当前两台相机的 `intrinsics.json` 和 ROS `camera_info.yaml`。
- `results/stereo_20260918_current/extrinsics.json`：从本地 7 组配对照片和上述两份内参重新计算的外参；其中输入路径是采集电脑上的历史路径。
- `data/checkerboard_50mm_10x7_600x450mm.pdf`：10×7 方格、9×6 内角点的打印棋盘。标称单格 50 mm，求绝对距离前应实测成品。
- `data/preview_DB2189859_slider_fit_20260917/` 与 `results/DB2189859/calibration_recommended_20260917/`：较早的 cam1 标定记录，保留供历史对照。

## 快速启动

在 Ubuntu 22.04 x86_64 上准备 MVS 5.0.1 SDK、`python3-opencv`、`python3-numpy` 和 `python3-pyqt5`。把 SDK 解压到 `$HOME/.local/opt/MVS-5.0.1`，或设置 `MVS_ROOT` 指向实际目录。**SDK 安装包与驱动未上传**；读取参数 JSON 不需要接相机，复算需要另行取得原图。

```bash
cd camera_intrinsics_mvs
./run.sh probe
./run.sh gui --serial DB2189859 --output-dir "$HOME/camera_calib_capture" \
  --intrinsics ../results/DB2189859/calibration_20260918_111338/intrinsics.json
```

上次正常采集时主机网口 `enp88s0` 的相机网段地址为 `192.168.1.88/24`，cam1 为 `192.168.1.213/24`，cam2 为 `192.168.1.214/24`。cam2 的地址曾由 MVS Force IP 临时设置，断电后可能改变；实际使用前以 `probe` 为准。**编写交接文档时该网口为 `NO-CARRIER`，所以这些是历史工作配置，不是当前在线状态。**

当前两份内参重投影 RMS 分别为 0.1150 px 和 0.0644 px。7 组外参的双目 RMS 为 0.1454 px、基线 160.78 mm、偏航角 -42.24°。外参组数少于推荐的 10～20 组，且与预期 60° 的安装角相差约 18°；应在最终安装后增加姿态并独立核查。取得原图后的复算命令与坐标变换方向见 [交接手册](HANDOFF_TWO_CAMERAS.md#6-取得原图后离线复算无需接相机)。

MVS SDK、SSH 私钥和相机密码不属于本仓库内容。
