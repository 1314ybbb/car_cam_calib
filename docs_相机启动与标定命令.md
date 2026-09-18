# 相机启动与标定命令

更新：2026-09-17。本机为 Ubuntu x86_64，MVS 5.0.1 已解压到 `/home/ybbb/.local/opt/MVS-5.0.1`，本项目脚本在 `/home/ybbb/codex_prj/camera_intrinsics_mvs`。

## 当前连接情况

- 网口：`enp88s0`，其中 `192.168.1.88/24` 与当前相机 `192.168.1.213/24` 同网段。
- 当前相机自报型号 **MV-CS023-10GC**、序列号 `DB2189859`、图像 `1920×1200`。最初计划标定的是 **MV-CS050-10GC**；换相机后先重新运行枚举，不要沿用当前序列号或内参。
- 当前成功取帧的方法使用 MVS SDK 动态库和普通网口。**不需要单独执行内核驱动启动命令**；此机未安装 MVS 的 `gevfilter` 内核模块。`run.sh` 会为 SDK 和 MVS 图形程序设置所需路径。
- MVS 图形上位机与 Python 预览都可能独占相机。切换软件前先在预览窗口按 `q` / `Esc`，或在 MVS 中关闭相机连接。

## 1. 检查网口并枚举相机

```bash
ip -br addr show enp88s0
ip -br link show enp88s0
cd /home/ybbb/codex_prj/camera_intrinsics_mvs
./run.sh probe
```

`probe` 列出型号、序列号、相机 IP 和主机网口 IP。同一相机因网口绑定多个 IP 可能重复出现；脚本取帧时优先选择与相机同子网的条目。网口应显示 `LOWER_UP`。

## 2. 启动实时预览与增益调节

每次指定一个**新的或空的**保存目录：

```bash
cd /home/ybbb/codex_prj/camera_intrinsics_mvs
./run.sh preview --serial DB2189859 \
  --output-dir "/home/ybbb/桌面/相机内参标定/preview_DB2189859_$(date +%Y%m%d_%H%M%S)"
```

预览窗口的 `Gain x0.1` 滑条可手动调节相机增益，画面上方显示实际回读值。预览图会缩放到屏幕内，点击窗口后按**空格**保存当前原始分辨率 PNG 和同名 JSON；`q` 或 `Esc` 退出。保存图像不含窗口上的文字。需要重新运行时，换一个空目录名；当前增益设置保留在相机中。当前曝光为 `5000 µs = 5 ms`。

只取一张诊断帧，无需预览窗口：

```bash
./run.sh snapshot --serial DB2189859 \
  --output "/home/ybbb/桌面/相机内参标定/diagnostic_$(date +%Y%m%d_%H%M%S).png"
```

## 3. 启动海康 MVS 图形上位机

先退出 Python 实时预览，再执行：

```bash
cd /home/ybbb/codex_prj/camera_intrinsics_mvs
./run.sh mvs
```

该入口使用本地解压的 MVS 程序，设置动态库与 Qt 插件路径；无需运行原安装包的 `setup.sh`。若从终端关闭图形程序，可在原终端按 `Ctrl+C`。`MVS` 包内的直接启动脚本 `bin/MVS.sh` 写死了 `/opt/MVS/lib/64`，本机解压路径不同，因此优先使用上述入口。

首次启动 MVS 时可能出现空白的引导浮层，点击右下角 **Skip All** 即可进入主界面。当前相机在左侧设备列表中出现；本地解压运行时终端会提示 `/opt/MVS/bin/Temp/Log` 日志路径不存在，图形界面仍可打开。正式标定取帧优先用上面的 Python 预览与采图命令。

## 4. 棋盘格照片与内参

棋盘格 PDF：`/home/ybbb/桌面/相机内参标定/checkerboard_50mm_10x7_600x450mm.pdf`。按 100% 打印，10×7 方格、9×6 内角点、标称单格 50 mm。实际打印后测量格距。每台相机与镜头组合单独采集；保持最终对焦、光圈、分辨率和 ROI。

连接计划标定的 **MV-CS050-10GC** 后，从 `./run.sh probe` 获取它的真实序列号，再采集约 25～35 张不同位置、距离和倾角的棋盘照片：

```bash
./run.sh capture --serial 目标相机序列号 --output /home/ybbb/桌面/相机内参标定/目标相机序列号/images --max-images 30
```

若这次要先标定当前 **MV-CS023-10GC**，在预览窗口按 `q` 退出后使用单独目录：

```bash
./run.sh capture --serial DB2189859 --expected-model MV-CS023-10GC \
  --output /home/ybbb/桌面/相机内参标定/DB2189859/images --max-images 30
```

3 米、棋盘只占画面约五分之一的照片可作为远距离样本；其余照片应在固定对焦下让棋盘更大，并分布到画面中心、四角和边缘，加入不同倾角。不要把同一位置的连续帧当作多种姿态。

建议拍 **30 张用于拟合，再另拍 8 张用于验证**。拟合照片按棋盘在图中的宽度安排：近距离约占画面宽度 35%～50% 拍 12 张，中距离约占 25%～35% 拍 12 张，当前约 3 米、占 15%～25% 拍 6 张。具体距离以棋盘清晰、能检出全部内角点为准。全组照片让棋盘中心覆盖画面中心、上下左右及四角附近；较近的照片以中心和边缘为主，较远的照片更容易放到角落。棋盘四周始终留完整白边。至少约 10 张让棋盘绕竖轴或横轴倾斜 20°～35°，左右与上下倾斜都要有；其余可正对或轻微倾斜。每次改变位置或角度后等棋盘静止再按空格；不要连续保存几张几乎相同的画面。

另拍 8 张验证照片，位置和倾角仍要变化，放在独立目录，且不要再用于拟合：

```bash
./run.sh capture --serial DB2189859 --expected-model MV-CS023-10GC \
  --output /home/ybbb/桌面/相机内参标定/DB2189859/validation_images --max-images 8
```

拍摄前固定最终使用的镜头对焦、光圈、曝光、增益、图像分辨率和 ROI。尽量让棋盘平整、无反光和过曝，内角点锐利；移动棋盘时不要拍。标称 50 mm 需用尺测量打印成品的实际格距。

采集窗口按空格保存能检出完整棋盘的帧；`q` / `Esc` 结束。用单独拍摄的验证照片检查内参：

```bash
./run.sh calibrate \
  --images /home/ybbb/桌面/相机内参标定/目标相机序列号/images \
  --validation-images /home/ybbb/桌面/相机内参标定/目标相机序列号/validation_images \
  --square-mm 50 \
  --output /home/ybbb/桌面/相机内参标定/目标相机序列号/result
```

`--square-mm` 填实测格距；`--validation-images` 可暂时省略。结果是 `intrinsics.json`、`camera_info.yaml` 和角点可视化。详细采图要求见 `/home/ybbb/codex_prj/camera_intrinsics_mvs/README.md`。

算法：OpenCV `findChessboardCornersSB` 检出每张照片的 9×6 亚像素内角点；`calibrateCameraExtended` 根据已知棋盘格尺寸拟合针孔相机内参矩阵 K，以及 5 个径向/切向畸变参数 `[k1,k2,p1,p2,k3]`，并给出每张照片的重投影误差。拟合至少需要 15 张有效照片，建议 25～35 张；检查 `intrinsics.json` 里的覆盖区域、倾角、距离及警告，并使用独立验证照片检查误差。当前未采到实际棋盘照片，不能先给出可信的内参数值。

## 5. 常用排查

```bash
ip -br addr show enp88s0       # 查看网口 IP
ip -br link show enp88s0       # 查看网线载波
./run.sh probe                  # 查看相机型号、序列号和 IP
lsmod | rg gevfilter            # 查看可选的 MVS 内核过滤驱动是否加载
```

`probe` 找不到相机时，先检查供电、网线、网口载波和 IP 网段。能枚举但无法打开时，先确认 MVS 图形程序或另一份预览没有占用相机。取帧全黑时，先检查镜头盖、光圈、光照、曝光和增益；诊断帧 JSON 记录了取帧参数及丢包情况。

## 6. 本次 34 张照片的计算结果（2026-09-17）

当前 **MV-CS023-10GC / DB2189859** 的 34 张预览原图均检出 9×6 内角点，分辨率 1920×1200，曝光 5 ms、增益约 15.006、无丢包。推荐结果在 `DB2189859/calibration_recommended_20260917`，包含 `intrinsics.json`、`camera_info.yaml`、逐张 CSV 与中文检查报告。该结果属于当前 MV-CS023-10GC，不能用于最初计划的 MV-CS050-10GC。

算法推导、模型选择、软件版本和本地脚本副本见 `DB2189859/calibration_recommended_20260917/内参标定算法说明.md`。

复现推荐的四个自由畸变参数模型（`k3` 固定为 0）：

```bash
cd /home/ybbb/codex_prj/camera_intrinsics_mvs
./run.sh calibrate \
  --images /home/ybbb/桌面/相机内参标定/preview_DB2189859_slider_fit_20260917 \
  --square-mm 50 --fix-k3 \
  --output /home/ybbb/桌面/相机内参标定/DB2189859/calibration_recommended_20260917
```

整体重投影 RMS 为 0.1163 px。角点横向仅覆盖画面宽度约 10.5%～79.3%，纵向约 16.8%～84.6%；全画面边缘，尤其右侧，仍建议补拍再标定。`50 mm` 为标称格距，若需物理尺度的外参距离，先测量打印成品。

## 7. 用一张棋盘照片估算距离

棋盘可放在不同位置、距离和倾角，只需完整清晰入镜。先拍一张**未用于拟合**的新照片，再执行：

```bash
cd /home/ybbb/codex_prj/camera_intrinsics_mvs
./run.sh distance \
  --image /home/ybbb/桌面/相机内参标定/DB2189859/validation_images/001.png \
  --intrinsics /home/ybbb/桌面/相机内参标定/DB2189859/calibration_recommended_20260917/intrinsics.json \
  --square-mm 50
```

替换为实际照片文件名及实测格距。输出同时包含**光心到棋盘中心的三维距离**和**沿光轴的深度**；卷尺从镜头前端量起与光心起点不同。程序副本和测距原理见 `DB2189859/calibration_recommended_20260917/内参标定算法说明.md`。

第 35 张（`frame_0035.png`）未参与内参拟合。计算结果：棋盘中心相机坐标约 `(-20.7, -123.1, 995.8) mm`，光心到棋盘中心距离 `1003.6 mm`，重投影 RMS `0.108 px`。完整结果保存在 `DB2189859/calibration_recommended_20260917/distance_frame_0035.json`。

第 36 张（`frame_0036.png`）同样未参与内参拟合。计算结果：棋盘中心约 `(-32.5, -105.1, 1310.3) mm`，光心到棋盘中心距离 `1314.9 mm`，重投影 RMS `0.067 px`；比第 35 张沿光轴远 `314.5 mm`。完整结果保存在 `DB2189859/calibration_recommended_20260917/distance_frame_0036.json`。
