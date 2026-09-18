# 相机启动与标定命令速查

完整环境、网络、数据和质量判断见 [双相机交接手册](HANDOFF_TWO_CAMERAS.md)。以下命令在仓库 `camera_intrinsics_mvs/` 目录执行。

## 接线与枚举

两台相机供电并接入千兆交换机，交换机连接电脑网口。上次工作配置为 cam1 `DB2189859 → 192.168.1.213/24`，cam2 `DB2189878 → 192.168.1.214/24`，主机 `enp88s0 → 192.168.1.88/24`。cam2 使用过临时 Force IP；实际地址以枚举结果为准。本机 2026-09-18 编写交接时网口无链路，需先恢复 `LOWER_UP`。

```bash
ip -br link show enp88s0
ip -br addr show enp88s0
cd camera_intrinsics_mvs
./run.sh probe
```

当前数据来自 **MV-CS023-10GC**，不是原计划的 MV-CS050-10GC。若接入其他型号或更换镜头，请重新标定。

## 上位机

只运行一个上位机或 MVS 程序，避免相机被独占而报 `0x80000203`。MVS SDK 默认位于 `$HOME/.local/opt/MVS-5.0.1`；其他路径可先 `export MVS_ROOT=/实际/MVS/目录`。

```bash
./run.sh gui --serial DB2189859 --output-dir "$HOME/camera_calib_capture" \
  --intrinsics ../results/DB2189859/calibration_20260918_111338/intrinsics.json
```

连接相机后，普通采集按物理位置进入 `cam1_parm_inside_DB2189859/` 或 `cam2_parm_inside_DB2189878/`。勾选“外参配对采集”后进入 `cam1_parm_outside_DB2189859/`、`cam2_parm_outside_DB2189878/`，同时显示双画面。棋盘固定时按空格保存同组两张照片，然后再移动棋盘并点“下一组”。界面的文件夹选择器已使用 Qt 目录窗口；若运行的是旧窗口，关闭后重新启动。

## 取得原图后离线复算

**2026-09-18 的原始照片没有上传 GitHub。**以下命令需要另行取得采集电脑的四个照片目录，并把 `CAPTURE_DESKTOP` 改为照片共同根路径；无需接相机，输出目录必须尚不存在。

```bash
CAPTURE_DESKTOP="/path/to/received_capture_desktop"
./run.sh calibrate --images "$CAPTURE_DESKTOP/cam1_parm_inside_DB2189859" \
  --square-mm 50 --fix-k3 --output /tmp/cam1_intrinsics_check
./run.sh calibrate --images "$CAPTURE_DESKTOP/cam2_parm_inside_DB2189878" \
  --square-mm 50 --fix-k3 --output /tmp/cam2_intrinsics_check
./run.sh stereo \
  --camera1-images "$CAPTURE_DESKTOP/outside/cam1_parm_outside_DB2189859" \
  --camera2-images "$CAPTURE_DESKTOP/outside/cam2_parm_outside_DB2189878" \
  --intrinsics1 ../results/DB2189859/calibration_20260918_111338/intrinsics.json \
  --intrinsics2 ../results/DB2189878/calibration_20260918_103029/intrinsics.json \
  --square-mm 50 --expected-angle-deg 60 --output /tmp/stereo_check
```

`extrinsics.json` 满足 `p_cam2 = R_camera1_to_camera2 × p_cam1 + T_camera1_to_camera2_mm`；60° 仅用于结果核查，不约束求解。当前只有 7 组，外参结果仍需增加姿态和核对真实安装角。
