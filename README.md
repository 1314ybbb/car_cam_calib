# Hikrobot GigE 相机内参标定工程

本仓库保存海康机器人 GigE 相机的取帧、棋盘格采集、OpenCV 内参标定和棋盘测距工具。

## 图形上位机

```bash
cd camera_intrinsics_mvs
./run.sh gui --serial DB2189859
```

需要本机安装 PyQt5、OpenCV 和 MVS SDK；这台机器已具备运行环境。在窗口中先选相机并连接，再为**每台相机选择独立的图片文件夹**。点击“保存图片 + JSON”或按空格，保存原始分辨率 PNG 与同名元数据 JSON。拍摄至少 15 张、建议 25～35 张清晰且姿态各异的 9×6 内角点棋盘照片，输入打印成品的实测格距，点击“一键生成内参”。结果写入图片文件夹下带时间戳的 `calibration_*/intrinsics.json`，并自动载入。也可以手动选择已有的内参 JSON，再打开“实时位姿”，查看棋盘坐标轴、棋盘中心相机坐标、光心距离和重投影 RMS。关闭开关后恢复普通预览。

图形程序与命令行预览都独占相机；请先退出当前预览再连接图形程序。所选内参会核对相机型号、序列号、分辨率和棋盘规格。实时位姿的坐标单位取自内参 JSON 的 `square_mm`，物理距离精度取决于实际打印格距及内参质量。

增益现在可拖动滑条或在“输入”框直接填写数值；按回车或点击“应用增益”后，旁边会以四位小数显示相机实际回读值。相机若只支持离散档位，设定值与回读值可能略有差异。连接相机后，程序会查找对应型号、序列号、分辨率的本地内参；也可点“查找已有内参”从带 RMS 的列表中选择。当前 `DB2189859` 的推荐文件是 `results/DB2189859/calibration_recommended_20260917/intrinsics.json`。

双相机棋盘格外参：两台相机先各自求好内参，分别选择图片文件夹。勾选“外参配对采集”，同一组号下**固定棋盘**，分别保存两台相机的照片；两张都拍完后再点击“下一组”并改变棋盘姿态。组与组之间可以移动棋盘。拍 10～20 组后打开“双相机棋盘格外参…”，填入两台相机的图片目录和 `intrinsics.json`，输入实测方格边长并计算。若配对一致性检查通过，结果写入相机 1 图片目录下的 `stereo_*/extrinsics.json`，包含相机 1 到相机 2 的 `R`、`T`、基线、偏航角、重投影误差和逐组检查结果；不通过时只写 `pair_review.json` 供查错。至少需要 5 组完整有效照片；两台相机可使用不同分辨率，但各自照片须与各自内参匹配。当前实现使用完整 9×6 棋盘格。手持或运动棋盘需要双相机同步触发，手动组号和几何检查无法保证照片真的是同一瞬间的棋盘位姿。

## 当前数据

本次实际采集相机是 **MV-CS023-10GC**，序列号 `DB2189859`，图像尺寸 `1920×1200`。最初计划的型号是 MV-CS050-10GC，两者的内参不能混用。

- `camera_intrinsics_mvs/`：Python/MVS SDK 工具和启动脚本
- `data/preview_DB2189859_slider_fit_20260917/`：34 张原始 PNG 及同名拍摄元数据 JSON
- `data/checkerboard_50mm_10x7_600x450mm.pdf`：10×7 方格（9×6 内角点）棋盘
- `results/DB2189859/calibration_recommended_20260917/`：`intrinsics.json`、ROS `camera_info.yaml`、逐张质量报告和测距结果
- `docs_相机启动与标定命令.md`：网口、预览、采集、标定和测距命令
- `CALIBRATION_HANDOFF_LINUX.md`：环境交接记录

推荐内参结果的整体重投影 RMS 为 0.1163 px。结果对应 1920×1200、曝光 5 ms、增益约 15 的当前相机配置；改变相机、镜头对焦、ROI、分辨率或装配后应重新标定。

## 快速使用

```bash
cd camera_intrinsics_mvs
./run.sh probe
./run.sh preview --serial DB2189859 --output-dir /tmp/camera_preview
```

MVS SDK 未复制进仓库。请从海康机器人官方 MVS 5.0.1 安装包安装，或将 SDK 解压到 `~/.local/opt/MVS-5.0.1`；`run.sh` 会自动使用该路径。SDK 和相机驱动属于第三方软件，不应提交到本仓库。

## 计算内参

```bash
./run.sh calibrate \
  --images ../data/preview_DB2189859_slider_fit_20260917 \
  --square-mm 50 --fix-k3 \
  --output ../results/reproduced
```

算法使用 `findChessboardCornersSB` 检出 9×6 亚像素角点，再用 `calibrateCameraExtended` 拟合针孔模型的 `K` 和 `[k1,k2,p1,p2,k3]`；推荐命令固定 `k3=0`。格距请用打印成品实测值替换 50 mm。

用一张新棋盘照片测距：

```bash
./run.sh distance \
  --image /path/to/frame.png \
  --intrinsics ../results/DB2189859/calibration_recommended_20260917/intrinsics.json \
  --square-mm 50
```

不要提交 SSH 私钥、MVS 安装包或相机密码。
