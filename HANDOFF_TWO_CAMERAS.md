# 双相机标定工程交接（2026-09-18）

本文件对应仓库中的 **两台已采集的 MV-CS023-10GC**。原计划型号为 **MV-CS050-10GC**，两者不同；仓库内参数不能直接用于 MV-CS050-10GC，也不能用于换镜头、重新对焦、改变 ROI 或重新安装后的相机。项目所述镜头为 MVL-KF0814M-12MPE（8 mm、C 口），但现有照片元数据不记录逐台镜头序列号，接手时应现场确认两台的镜头及对焦状态。

## 1. 接手先看这里

1. 将两台相机接到千兆交换机，再把交换机接到电脑网口 `enp88s0`，并给相机供电。**2026-09-18 编写本文时该网口为 `NO-CARRIER`，下表 IP 是先前成功采集时记录的值，并非当前在线验证。**
2. 在 Ubuntu 22.04 x86_64 上准备 MVS 5.0.1 SDK 和 Python 依赖；按第 3 节执行 `./run.sh probe`，核对型号、序列号和 IP。
3. **只开一个**本工程上位机或 MVS 图形程序。连接相机，选保存根目录，采集或载入对应内参；双相机外参按同组号保存两幅静止棋盘照片。
4. 核对仓库中的两份内参、外参和采集摘要。按用户要求，**2026-09-18 原始照片未上传**；如另行取得照片，可用第 6 节命令复算。当前外参只有 7 组且与预期 60° 相差约 18°，应增加配对姿态、核实安装角后再用于正式系统。

## 2. 硬件、物理位置与网络

| 物理位置 | 相机序列号 | 相机自报型号 | 上次采集 IP | 保存目录前缀 |
|---|---|---|---|---|
| cam1 | `DB2189859` | `MV-CS023-10GC` | `192.168.1.213/24` | `cam1_parm_*_DB2189859` |
| cam2 | `DB2189878` | `MV-CS023-10GC` | `192.168.1.214/24` | `cam2_parm_*_DB2189878` |

- 上次正常采集时，主机 `enp88s0` 上的相机网段地址是 `192.168.1.88/24`。这块网卡还可能承载其他传感器网段，**不要覆盖或清空它的其他地址**。交换机与两台相机均应通电，`ip -br link show enp88s0` 应显示 `LOWER_UP`。
- cam2 的 `.214` 曾通过 MVS **Force IP** 临时设置；断电后可能回到 `169.254.*`。在 MVS 的 **Tool → IP Configurator** 中把它设置到与主机相同的 `192.168.1.*` 网段，并确保不与 cam1 或主机重复。必要时配置持久 IP；完成后重新运行 `probe`。
- 当前机器的 NetworkManager 连接名是 `有线连接 1`，其中配置了 `192.168.1.88/24` 以及其他传感器地址。队友电脑的网卡名称和连接名可能不同，以自己的 `ip -br addr` 为准。若仅需临时增加相机网段地址，可在网线连通后检查无重复地址，再执行 `sudo ip addr add 192.168.1.88/24 dev 实际网卡名`；重连后临时地址可能消失。
- 棋盘为 10×7 方格、9×6 内角点；仓库 `data/checkerboard_50mm_10x7_600x450mm.pdf` 按 100% 打印。当前求解输入的单格尺寸为 **50.000 mm**。应测量实体成品；格距错误会按比例影响平移和距离。

## 3. 软件环境与启动

本机验证环境：Ubuntu 22.04.5 LTS、Python 3.10.12、系统 OpenCV 4.5.4、NumPy 1.21.5、PyQt5 5.15.6。`run.sh` 固定使用 `/usr/bin/python3`，并优先加载 `/usr/lib/python3/dist-packages`；直接运行 Python 可能选择用户目录的另一版 OpenCV。MVS 软件包标称 5.0.1，照片 JSON 记录的 SDK 内部版本为 `0x04080003`。

```bash
git clone git@github.com:1314ybbb/car_cam_calib.git
cd car_cam_calib/camera_intrinsics_mvs
sudo apt install python3-opencv python3-numpy python3-pyqt5
```

从海康机器人取得与系统架构匹配的 MVS SDK，把它安装或解压到 `$HOME/.local/opt/MVS-5.0.1`，确认存在 `lib/64/libMvCameraControl.so` 和 `Samples/64/Python/MvImport/`。SDK/驱动压缩包未提交到 GitHub。若使用其他位置，可 `export MVS_ROOT=/实际/MVS/目录`。本机用普通网口与 SDK 取帧，无须先启动单独的内核驱动。

```bash
ip -br link show enp88s0              # 队友机器替换为自己的网卡名
ip -br addr show enp88s0
./run.sh probe                         # 列出型号、序列号、相机 IP、主机接口 IP
./run.sh gui --serial DB2189859 --output-dir "$HOME/camera_calib_capture" \
  --intrinsics ../results/DB2189859/calibration_20260918_111338/intrinsics.json
```

上位机中按“连接相机”；选保存根目录后，普通采集自动进入 `camX_parm_inside_序列号/`，勾选“外参配对采集”后进入 `camX_parm_outside_序列号/`。空格或“保存图片 + JSON”保存原分辨率 PNG 和同名元数据 JSON。两台相机都连接且勾选配对时显示左右画面、每台临时限制到 6 fps，并设置约 50 µs GigE 包间隔；退出双画面恢复原帧率和包间隔。第二台可单独调增益。窗口里的“查找已有内参”按序列号筛选；队友可直接选第 5 节所列的两份 `intrinsics.json`。仓库还保留旧版 cam1 内参，因此启动命令明确指定本次使用的 cam1 文件，避免自动选择历史结果。

## 4. 采集规则

- **内参**：每台相机单独拍 25～35 张清晰完整的棋盘，变化距离、图像位置和左右/上下倾角。当前数据 cam1 为 36 张、cam2 为 30 张；不要把不同序列号照片混在一处。
- **外参**：固定棋盘后，同一组号下两台相机各拍一张；拍完两张才移动棋盘并按“下一组”。建议 10～20 组不同棋盘姿态。软件配对只保证最近帧接收时间差不超过 300 ms，**不等于硬件同步曝光**；棋盘必须在这一组两次曝光期间静止。
- 当前四组目录中的所有照片均为 1920×1200，曝光 `5000 µs`，相机回读增益约 `15.0062`，记录的帧丢包数为 0；这只是已保存照片的状态，不保证后续链路无丢包。相机 1/2 的 IP 与主机接口 IP 已写入每张照片同名 JSON。
- 如镜头、对焦、相机姿态、分辨率或 ROI 变化，应重新验证或重新标定。将本次实验室外参用于车上之前，必须在最终安装状态下重新采集并验证。

## 5. 本地原图与仓库中的参数

| 用途 | 位置 | 数量或结果 |
|---|---|---|
| cam1 内参原图与 JSON | 采集电脑 `/home/ybbb/桌面/cam1_parm_inside_DB2189859/`，**未上传** | 36 对 |
| cam2 内参原图与 JSON | 采集电脑 `/home/ybbb/桌面/cam2_parm_inside_DB2189878/`，**未上传** | 30 对 |
| cam1 外参原图与 JSON | 采集电脑 `/home/ybbb/桌面/outside/cam1_parm_outside_DB2189859/`，**未上传** | 组号 1～7 |
| cam2 外参原图与 JSON | 采集电脑 `/home/ybbb/桌面/outside/cam2_parm_outside_DB2189878/`，**未上传** | 组号 1～7 |
| cam1 当前内参 | `results/DB2189859/calibration_20260918_111338/intrinsics.json` | 36 张，RMS 0.1150 px |
| cam2 当前内参 | `results/DB2189878/calibration_20260918_103029/intrinsics.json` | 30 张，RMS 0.0644 px |
| 用上述数据复算的外参 | `results/stereo_20260918_current/extrinsics.json` | 7 组，双目 RMS 0.1454 px |
| 采集配置和 IP 摘要 | `results/capture_manifest_20260918.json` | 四组照片数量、相机型号、IP、曝光、增益、丢包与两帧时间差 |

仓库原先已有 `data/preview_DB2189859_slider_fit_20260917/` 与 `results/DB2189859/calibration_recommended_20260917/`，它们是 **较早的 cam1 数据和结果**，不能代替本次 cam2 照片或外参配对照片。当前 `extrinsics.json` 的 `images`、`intrinsics` 字段记录采集电脑上的原始路径；换电脑后需指向队友自己的照片路径才能复算。

当前外参从 cam1 到 cam2：`p_cam2 = R_camera1_to_camera2 · p_cam1 + T_camera1_to_camera2_mm`。计算出的基线 `160.78 mm`、偏航角 `-42.24°`、三维旋转角 `43.19°`；`camera2_center_in_camera1_mm` 是 cam2 光心在 cam1 坐标系中的位置。用户输入的 **60°** 只用于报警，不参与拟合。当前 7 组通过程序几何一致性检查，但少于推荐的 10～20 组，且光轴夹角与预期相差约 18°；要核对实际安装方向并增加姿态，不把低 RMS 单独当成安装角正确的证据。

## 6. 取得原图后离线复算（无需接相机）

**仅克隆 GitHub 仓库无法运行以下命令，因为 2026-09-18 的照片没有上传。**如另行取得第 5 节四个目录，保持其中的 PNG 与同名 JSON，并把 `CAPTURE_DESKTOP` 改成它们的共同根路径。在 `camera_intrinsics_mvs/` 中执行；输出目录必须是尚不存在的新目录。

```bash
CAPTURE_DESKTOP="/path/to/received_capture_desktop"
./run.sh calibrate \
  --images "$CAPTURE_DESKTOP/cam1_parm_inside_DB2189859" \
  --square-mm 50 --fix-k3 --output /tmp/cam1_intrinsics_check
./run.sh calibrate \
  --images "$CAPTURE_DESKTOP/cam2_parm_inside_DB2189878" \
  --square-mm 50 --fix-k3 --output /tmp/cam2_intrinsics_check
./run.sh stereo \
  --camera1-images "$CAPTURE_DESKTOP/outside/cam1_parm_outside_DB2189859" \
  --camera2-images "$CAPTURE_DESKTOP/outside/cam2_parm_outside_DB2189878" \
  --intrinsics1 ../results/DB2189859/calibration_20260918_111338/intrinsics.json \
  --intrinsics2 ../results/DB2189878/calibration_20260918_103029/intrinsics.json \
  --square-mm 50 --expected-angle-deg 60 --output /tmp/stereo_check
```

内参算法用 OpenCV `findChessboardCornersSB` 检出 9×6 亚像素角点，再用 `calibrateCameraExtended` 拟合针孔模型 `K` 和 `[k1,k2,p1,p2,k3]`（当前固定 `k3=0`）。双相机用 `stereoCalibrate(CALIB_FIX_INTRINSIC)` 固定两份内参求相对旋转和平移，按组号检查。`extrinsics.json` 中的相机身份必须与各自内参和照片 JSON 一致；两台相机顺序互换时 R/T 的方向也会变。

## 7. 常见故障

| 现象 | 先检查 |
|---|---|
| `probe` 找不到相机 | 电源、交换机、网线、网口 `LOWER_UP`；再检查主机与相机 IP 是否同网段。 |
| 能枚举但 `open camera failed: 0x80000203` | MVS 定义为“设备无访问权限”。常见于同一相机被另一上位机/MVS 窗口独占；关闭重复进程后重连。 |
| cam2 重启后不能打开 | 检查 `.214` 是否只是 Force IP 临时地址，必要时用 MVS IP Configurator 重新设置。 |
| 双画面丢包或帧不完整 | 看窗口两路丢包计数；检查千兆交换机/网线与主机网口，程序双画面默认 6 fps 和约 50 µs 包间隔。 |
| “选择图片文件夹”卡顿 | 最新代码已改用 Qt 目录选择器；旧窗口需要关闭后重启才会加载新代码。 |
| 外参角度与 60° 不符 | 核实物理光轴夹角的测量定义、相机 1/2 顺序和配对时棋盘静止；检查逐组角点与增加采集组数。 |

本仓库含代码、参数、采集摘要和棋盘 PDF；仓库历史中已有 2026-09-17 的 cam1 照片，**此次未新增照片，也未上传 2026-09-18 的四组照片**。仓库不含 MVS SDK/驱动安装包或相机硬件配置备份。首次移交应由队友在自己电脑上完成一次 `probe`、单帧保存和现场双相机采集；拿到原图后再复算，并记录自己的网卡名称、实际 IP、镜头状态与新标定结果。
