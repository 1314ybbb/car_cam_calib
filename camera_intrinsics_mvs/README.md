# MV-CS050-10GC 内参标定

这套脚本使用随项目提供的 MVS 5.0.1 SDK 读取 GigE 相机，并用普通棋盘格估计 OpenCV 透视模型的 5 个畸变系数。每台相机和镜头组合单独采集、单独求参。当前机器上 SDK 已放在 `~/.local/opt/MVS-5.0.1`，无须修改系统驱动；`run.sh` 选择本机带 GTK 的 OpenCV 4.5.4。

## 1. 接线、试读

给相机供电，使用网线连接到本机以太网口，再执行：

```bash
cd /home/ybbb/codex_prj/camera_intrinsics_mvs
./run.sh probe
```

`probe` 会列出型号、序列号和 IP。若显示 `No GigE camera found`，先看 `ip -br link` 中以太网口是否为 `LOWER_UP`，检查供电、网线和相机 IP。若能枚举但无法打开，再检查相机和网口是否在同一子网；不要把系统自带笔记本摄像头 `/dev/video0` 当作这台 GigE 工业相机。相机实际最大分辨率见海康官网产品页，采集时以相机当前 `Width`、`Height`、ROI 为准。

先保存一张诊断帧，无须打开预览窗口：

```bash
./run.sh snapshot --serial 相机序列号 --output /home/ybbb/桌面/相机内参标定/diagnostic.png
```

同名 JSON 会记录设备型号、序列号、相机参数、像素格式、丢包数和图像亮度。若枚举列表重复出现同一序列号，脚本会优先选择与相机同子网的主机 IP。`snapshot` 可诊断任何已枚举的 GigE 相机；正式 `capture` 默认只接受项目指定的 `MV-CS050-10GC`。若要明确标定其他型号，必须传入 `--expected-model` 并用另一目录保存。

实时预览可用 `./run.sh preview --serial 相机序列号 --output-dir /home/ybbb/桌面/相机内参标定/preview_相机序列号` 启动。窗口内的 `Gain x0.1` 滑条会直接调节相机手动增益，并显示实际回读值；预览按窗口尺寸缩放，保存仍为原始分辨率。点击预览窗口后，按空格保存 PNG，按 `q` 或 `Esc` 退出；保存的图像不会含预览文字。同名 JSON 记录保存时的相机参数。预览可用于任何已枚举相机，保存的照片不自动作为棋盘标定样本。

## 2. 准备棋盘格并拍照

使用 `/home/ybbb/桌面/相机内参标定/checkerboard_50mm_10x7_600x450mm.pdf`，以 **100% 实际尺寸**打印。图案为 10×7 方格、9×6 内角点。贴在平整硬板上，测量单格节距并记录实测值。`--square-mm` 以后应填实测值。

保持镜头最终使用的焦距、对焦、光圈和相机的分辨率、ROI、binning 不变。避免过曝、运动模糊和高反光。先让整块棋盘清晰入镜，建议获取 25～35 张不同姿态的照片：中央、四角、边缘；正对、左右倾斜、上下倾斜；近、中、远距离。棋盘尽量覆盖较大画面，但所有 54 个内角点需可见。两次按空格之间移动板并停稳；避免连拍几乎相同的姿态。可先用少量照片试拍检测。

```bash
./run.sh capture --serial 相机序列号 --output /home/ybbb/桌面/相机内参标定/相机序列号/images --max-images 35
```

预览窗口中空格保存，`q` 或 `Esc` 结束。只有棋盘成功检出且该帧未丢包时保存无损 PNG。采集目录同时保存 `capture_manifest.json`，含序列号、相机设置、帧号、像素格式和图像转换方式。多台相机务必指定 `--serial` 并使用不同目录。

## 3. 求参和验证

建议另拍 5～10 张新姿态图，放在独立的 `validation_images` 目录中；目录中只放图像。求参：

```bash
./run.sh calibrate \
  --images /home/ybbb/桌面/相机内参标定/相机序列号/images \
  --validation-images /home/ybbb/桌面/相机内参标定/相机序列号/validation_images \
  --square-mm 50 \
  --output /home/ybbb/桌面/相机内参标定/相机序列号/result
```

`--validation-images` 可暂时省略。结果含 `intrinsics.json`、ROS `camera_info.yaml`、`detected_corners/` 可视化。JSON 给出像素单位 `K`、顺序为 `k1,k2,p1,p2,k3` 的 `D`、整体及逐图重投影误差、覆盖范围和警告；验证图仅用于单张位姿拟合与重投影核查，不参与内参拟合。重投影误差只是内部几何一致性指标，还要检查去畸变后的直线和独立场景。固定对焦、ROI、binning 或镜头装配改变后应重新验证或重标定。

算法：OpenCV `findChessboardCornersSB` 检测亚像素角点；`calibrateCameraExtended` 用 5 系数针孔/径向/切向模型全局最小化重投影误差。该 8 mm 定焦镜头不按鱼眼模型预设。至少需要 15 张有效图，推荐 25～35 张且覆盖充分；脚本不自动删高误差图，需检查可视化后决定是否重拍。
