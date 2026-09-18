# 车载多传感器标定：Linux端交接记忆

整理日期：2026-09-17。本文为本次对话中标定内容的结构化交接记录，不是逐字聊天导出。内容区分用户确认事实、讨论建议和未验证事项；不得把建议当成已完成实验。

## 1. 项目目标与协作约束

- 项目以工程落地为主，不要求算法创新。
- 已购车架、八卡服务器、各类相机和雷达；用户表示各传感器上位机及几个融合算法已分别跑通。不能据此认为已完成同步采集或联合标定。
- 所有传感器安装在车顶支架上，计划全传感器共同采集，不采取先删减成最小传感器组合的路线。
- 阶段目标是车上实现识别效果，不要求12周内整个项目结束，尚未设定定量验收指标。
- 三模态指相机、激光雷达、毫米波雷达：全都采集和标注；三种双模态融合至少跑通两种，三模态融合为进阶目标。
- 主要难点：内外参标定、点云与图像对齐、多模态2D/3D关联标注。
- 用户队友负责复现Koide无靶标相机—LiDAR标定。用户本人调查Fraunhofer、TU Delft，之后又考虑FAST-Calib。
- 目前没有用户确认的最终工具选型，也没有用户确认的实机标定成功结果。

## 2. 用户确认的硬件

| 设备 | 型号/用途 | 状态与注意事项 |
|---|---|---|
| 环视工业相机 | 海康机器人 MV-CS050-10GC | 多台；准确数量、感光尺寸、分辨率/ROI尚待核实。不要擅自替换成PRO型号 |
| 工业镜头 | MVL-KF0814M-12MPE | 用户提供包装照片确认：8 mm、F1.4、1.1英寸、C口 |
| 前视LiDAR | 禾赛AT128 | 已安装 |
| 近距补盲LiDAR | 禾赛QT128 | 已安装；不得默认安装后360°无遮挡 |
| 前视毫米波雷达 | 大陆ARS548 | 4D雷达，需要适配三维检测观测 |
| 前视偏振相机 | 型号未知 | 不默认使用上述海康镜头；成像结构也未确认 |

用户提供车顶支架CAD效果图：传感器分散在框架四周，有横梁、支座等结构；没有标注设备身份和光轴，因此不能仅凭模型颜色识别设备。需实测共同视场、遮挡及支架稳定性。

镜头官方规格：8 mm定焦，手动对焦，光圈F1.4～F16，像面直径17.6 mm，C-Mount，标称畸变5.5%。规格表中的水平84.8°对应14.14 mm像面宽度，不能直接套用为实际机身视场。5.5%不能直接作为OpenCV畸变系数。

镜头资料：https://www.hikrobotics.com/cn2/source/vision/document/2023/4/15/MVL-KF0814M-12MPE_20221022.pdf

## 3. 已对齐的标定原则

1. 相机优先采用普通透视模型加径向/切向畸变，不因用途为环视就默认鱼眼；用独立数据判断模型是否足够。
2. 每台机身与实际镜头组合分别标定。固定对焦、光圈、图像尺寸、ROI和重建/去畸变流程。
3. 不为近处标定板重新对焦、标完又调回远处。必要时使用更大板、放得更远。
4. 外参应在最终上车紧固后测定。CAD、尺子和水平仪用于初值、约束及独立检查，不直接等同于最终数据坐标系外参。
5. 水平仪不能独立确定偏航角；设备壳体原点、安装孔与光心/测量坐标系也不一定重合。
6. 静止数据标定不能解决动态采集的时间偏差。需另外核查设备时间戳、主机时间、触发与同步方式，不默认全部设备支持同一种PTP方案。
7. 同时录制所有传感器不等于全部必须同时看到一块板。可以按共同视场分组、围车移动靶标。
8. 参数统一明确变换方向，例如 `p_camera = T_camera_lidar * p_lidar`。必须核查每个开源项目的实际约定。
9. 输出参数必须绑定原始数据、配置、软件版本和安装状态；换场景独立验证，不仅看求解残差。
10. 不把点云到图像的深度相关投影等同于一张通用单应矩阵；不把全景拼接当作完整几何标定。

候选连接关系（不是已确认安装拓扑）：QT128连接环视相机，AT128与QT128标定，ARS548连接前视可共同观测的LiDAR，偏振相机连接前视普通相机或LiDAR。最终参考系和连接边需按真实共同视场决定。

## 4. 工具选型与准确边界

### 4.1 Fraunhofer multisensor_calibration

- 代码：https://github.com/FraunhoferIOSB/multisensor_calibration
- 文档：https://fraunhoferiosb.github.io/multisensor_calibration/
- 操作：https://fraunhoferiosb.github.io/multisensor_calibration/tutorial/
- 靶标：https://fraunhoferiosb.github.io/multisensor_calibration/calibration_target/
- 论文：Multi-Sensor Calibration Toolbox for Large-Scale Offroad Robotics，2025。
- PDF：https://robdekon.de/user/data/bibliography/pdf/Ruf2025a.pdf
- 备用PDF：https://publica-rest.fraunhofer.de/server/api/core/bitstreams/df275a1d-02a8-42e8-9dc8-cf2fad30f757/content

定位：有靶标外参工具，适用于相机—LiDAR、LiDAR—LiDAR，以及传感器对参考坐标系标定。ROS 2主线，ROS 1另有分支。不是ARS548或全传感器内参工具。

原理：图像识别ArUco角点；LiDAR拟合板几何、推算标记三维位置；相机—LiDAR使用2D/3D对应求PnP。双LiDAR使用靶标观测与GICP配准。内参需预先准备。

默认板（以靶标专页为准）：

- 宽1.2 m、高0.8 m。
- **三个非对称圆孔，不是四孔板**；孔半径0.12 m。
- 圆孔中心相对板中心为(-0.15,+0.15)、(+0.15,-0.15)、(-0.15,-0.15) m。
- 四个ArUco，字典 `DICT_6X6_250`，从左上顺时针ID为1、2、3、4，码边长0.18 m。
- 官方支持编辑靶标YAML中的板尺寸、标记ID/位置/大小和圆孔参数；**还需同步匹配CAD mesh/cloud**，用于姿态细化。
- 文档中GUI靶标项“不可编辑”不代表不能自定义文件。实际路径按仓库版本核查，不照抄文档拼写错误。
- 对称四孔板可以探索适配，但不能默认直接兼容；需检查LiDAR端方向歧义。

对本项目：保留为双LiDAR主候选；相机—LiDAR可与Koide或FAST-Calib比较。AT128/QT128实机兼容尚未验证。

### 4.2 TU Delft multi_sensor_calibration

- 代码：https://github.com/tudelft-iv/multi_sensor_calibration
- 靶标：https://github.com/tudelft-iv/multi_sensor_calibration/blob/master/docs/calibration_board.md
- 检测接口：https://github.com/tudelft-iv/multi_sensor_calibration/blob/master/docs/detectors.md
- 2019论文：An Extrinsic Calibration Tool for Radar, Camera and Lidar，ICRA。
- PDF：https://intelligent-vehicles.org/wp-content/uploads/2019/05/domhof2019icra_ext_sensor_calib.pdf
- 备用：https://repository.tudelft.nl/file/File_2e7db4ce-d2c5-464b-bb89-6374f7fbb59e
- 2021扩展：A Joint Extrinsic Calibration Tool for Radar, Camera and Lidar，IEEE TIV。
- 下载页：https://repository.tudelft.nl/record/uuid%3A285bce82-4cc3-453a-ace0-45688d339203

定位：相机、LiDAR、毫米波雷达共同靶标与联合外参求解框架。原环境为Ubuntu16.04/18.04、ROS Kinetic/Melodic，需隔离复现或移植。

模块：各模态检测器 → 观测累积 → 优化器 → YAML/URDF输出。

原版靶标：泡沫板1.5×1.0 m，四个圆孔，孔径15 cm、中心间距24 cm，背后一个三面角反射器；反射器相对板正面的设计偏移10.5 cm。孔有斜切设计。角反射器实际参考点应按图纸/代码定义，不能拿板中心代替。

允许改尺寸：同步修改检测器配置和 `optimization/src/optimization/calibration_board.py` 等模型。不能只改实物。

求解策略：MCPE（最小连接）、FCPE（全连接与闭环一致性）、PSE（位姿及靶标结构联合估计）。

关键限制：

- 原论文使用二维雷达观测；**不是已验证的ARS548三维即插即用方案**。
- 原LiDAR检测器使用Velodyne式点云及ring信息；禾赛适配可能不只是改字段。
- 必须区分“原版复现成功”与“ARS548适配成功”。
- 不建议为了套旧工具直接丢弃ARS548高度信息后宣称完成六自由度标定。

### 4.3 Koide direct_visual_lidar_calibration（队友负责）

- 代码：https://github.com/koide3/direct_visual_lidar_calibration
- 文档：https://koide3.github.io/direct_visual_lidar_calibration/
- 采集：https://koide3.github.io/direct_visual_lidar_calibration/collection/
- 程序：https://koide3.github.io/direct_visual_lidar_calibration/programs/
- 论文：General, Single-shot, Target-less, and Automatic LiDAR-Camera Extrinsic Calibration Toolbox，ICRA2023。

利用自然场景结构、纹理及LiDAR强度信息做相机—LiDAR外参。可作为正式主方案候选，不仅是复核工具；不支持直接替代双LiDAR或毫米波标定。

要求：相机内参先完成，传感器刚性固定，共同视场充分，强度字段有效；建议多个场景。Single-shot不意味着任意单帧都可靠，预处理可能包含点云累积。不要单独移动支架上的一台传感器套用手持采集示例。

自动初始化可用SuperGlue，亦可人工选对应点；SuperGlue存在单独许可限制。官方 `T_lidar_camera` 表示相机坐标变到LiDAR：`p_lidar = T_lidar_camera * p_camera`。

### 4.4 FAST-Calib（后续新增候选）

- 代码及图纸入口：https://github.com/hku-mars/FAST-Calib
- 论文名：FAST-Calib: LiDAR-Camera Extrinsic Calibration in One Second。

专门做相机—LiDAR外参，不需要先运行FAST-LIVO2。后者是里程计系统，商家所称“FAST-LIVO2标定板”不能当成规格证明。

用户展示商品图为四圆孔加四角视觉标记。可用于对应的外参方法，但实际尺寸、字典、ID、朝向、材料均需核对官方CAD。不能只凭图片确认完全匹配。

官方示例包含Mid360、Avia、Ouster；支持固态/机械LiDAR不等于AT128、QT128已验证。需检查输入与圆孔提取。

若已拥有匹配四孔板，可将FAST-Calib作为相机—LiDAR有靶标主线；Fraunhofer继续负责双LiDAR，ARS548单独处理。不是要求同时复现所有工具。

### 4.5 其他边界说明

- TIER IV marker_radar_lidar_calibrator明确支持ARS408式无俯仰角检测；ARS408不是通常所称4D成像雷达。**不要把TIER IV验证过ARS408，写成TU Delft已验证ARS408。**
- TIER IV说明：https://github.com/tier4/CalibrationTools/blob/tier4/universe/calibrators/marker_radar_lidar_calibrator/README.md
- 非重叠相机可研究标签式SfM，但需要连通的观测关系，不能独立移动板后凭空关联；LiDARTag对禾赛适配未验证。
- 偏振相机几何标定与偏振响应校正分开；若为微偏振片阵列结构，可探索固定重建后的总强度图做几何标定。型号未明，不把四个偏振方向当四台独立相机。
- polanalyser：https://github.com/elerac/polanalyser ，是图像处理辅助，不是完整偏振标定工具。

## 5. 角反射器与板材讨论结论

- 三面角反射器由相互垂直导电平面形成，在有效入射范围通过连续反射将能量返回来波方向；无源，不是主动放大。
- 装在板背后不等于开口背对雷达。开口仍朝向雷达，板材必须足够透波。
- 泡沫用于减小对毫米波观测的影响；材料含水、覆膜、金属支架等均需测试。
- 普通亚克力可透过部分毫米波，但会带来衰减、反射和传播变化；透明不等于适合LiDAR板面检测。不能未经验证直接替代原泡沫。
- 雷达罩参考：https://www.ti.com/lit/an/swra705/swra705.pdf
- 更大角反射器不必然更好；需要兼顾距离、方向性、回波强度和近场条件，不凭照片确定尺寸。
- 角反射器不是必须被雷达识别成三角形；检测可能是一点或若干邻近点，通过距离/方向门限、RCS、空背景对比、多帧稳定性和移位实验确定身份。
- 不能只取全场最强点，也不能仅凭零速度判断；静态杂波同样可能为零速度。
- 每次只放一个反射器并给摆放位置编号，降低关联复杂度。板位置变化后固定采集，人员离开，避免人体回波。
- 每个摆放位置得到一对参考点，多个位置求共同R、t。一处的一千帧不能替代空间分布不同的观测。
- 理想三维点对应至少三个不共线点可确定刚体变换；这不是原二维雷达模型的通用最低要求，也不是实际采集数量建议。
- TU Delft操作建议约至少10个靶标位置。针对ARS548增加距离、左右和高度变化，不全部同高共线。
- 先做裸反射器测试，再加板对比，再关联LiDAR板姿态，最后做优化。

## 6. 相机内参板规格与已生成文件

### 6.1 普通棋盘格

| 项目 | 标准版 | 大尺寸版 |
|---|---|---|
| 单格 | 50 mm | 80 mm |
| 方格数 | 10列×7行 | 10列×7行 |
| 内角点 | 9列×6行 | 9列×6行 |
| 图案 | 500×350 mm | 800×560 mm |
| 页面（含白边） | 600×450 mm | 900×660 mm |

OpenCV棋盘格 `patternSize=(9,6)`。优先试50 mm版，远处才清晰或操作不便再选80 mm版。约1～2 m只是早期试拍建议，不是按实际感光尺寸精算的限制；以工作对焦下清晰度和画面占比为准。

### 6.2 ChArUco（用户已要求生成PDF）

| 项目 | 标准版 | 大尺寸版 |
|---|---|---|
| 棋盘方格数 | 10×7 | 10×7 |
| squareLength | 50 mm | 80 mm |
| markerLength | 35 mm | 56 mm |
| 图案 | 500×350 mm | 800×560 mm |
| 页面 | 600×450 mm | 900×660 mm |
| 外围白边 | 四边50 mm | 四边50 mm |
| 字典 | DICT_5X5_100 | DICT_5X5_100 |
| 标记数量/ID | 35个，0～34 | 35个，0～34 |
| 棋盘内角点 | 54个 | 54个 |
| legacyPattern | false | false |
| markerBorderBits | 1 | 1 |

markerLength包含码自身黑边，不含外围白色间隔。ChArUco参数10×7是方格数，不是码数。Linux端必须用同字典、板式、ID顺序与尺寸；建议直接读取附带JSON，不凭记忆重建。

生成环境：Windows独立venv，OpenCV 5.0.0.93、NumPy 2.5.3。生成时已对源图案执行OpenCV识别，每份识别35标记与54角点；**未进行实体打印验证，亦未独立渲染PDF再识别**。

PDF为原尺寸矢量矩形图案，不是图片缩放。必须以实际大小/100%打印，禁用适合页面。制作后测量格距、检查平整度与反光。

与纯ArUco比较：纯ArUco可做内参，但标记角点通常不如棋盘角点精细；ChArUco结合ID与棋盘角点，方便局部可见和边缘采集，不保证无条件比优质棋盘格更准。

官方参考：https://docs.opencv.org/4.x/da/d13/tutorial_aruco_calibration.html

### 6.3 文件清单与迁移

Windows目录：`D:\codex_prj\calibration_printouts_20260917`

同目录文件：

- `checkerboard_50mm_10x7_600x450mm.pdf`
- `checkerboard_80mm_10x7_900x660mm.pdf`
- `charuco_10x7_square50mm_marker35mm.pdf`
- `charuco_10x7_square80mm_marker56mm.pdf`
- `charuco_10x7_square50mm_marker35mm.json`
- `charuco_10x7_square80mm_marker56mm.json`
- `generate_checkerboards.py`：标准库生成普通棋盘PDF。
- `generate_charuco.py`：OpenCV生成标准图案并检查识别，输出矢量PDF及JSON。
- 本文 `CALIBRATION_HANDOFF_LINUX.md`。

迁移时复制上述文件即可，**不要复制Windows的`.venv`到Linux使用**。本文包含关键参数，即使只迁移本文也可以恢复上下文，但PDF与脚本并未嵌入本文。

Linux需要重生成时，建议在新目录使用独立环境（先核实该环境的Python/OpenCV兼容性）：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install opencv-contrib-python-headless
python generate_checkerboards.py
python generate_charuco.py
```

记录实际安装版本。脚本在现有Windows环境运行成功；上述Linux命令尚未在目标机执行验证。不要覆盖唯一的已交付PDF备份。

### 6.4 手持采集

- 相机内参可手持刚性板：移动、停稳、拍照，不遮角点、不弯板、不在曝光期间晃动。
- 相机—LiDAR、双LiDAR尽量用支架，避免曝光与扫描时刻差造成板姿态不同。
- 毫米波联合标定不建议手持：固定后人员退出观测区。
- 全部使用实际采集的镜头状态，逐台标定；无需所有环视相机同时看板。

## 7. 建议复现路径（尚未执行）

### Fraunhofer

1. 固定版本、独立环境，官方示例完成启动、检测、采集、求解和保存闭环。
2. 接入一组图像、CameraInfo与点云，核对单位、坐标轴、时间、去畸变状态。
3. 按软件模型验证样板；样板仅用于检测验证，正式求参使用平整且尺寸准确的板。
4. 完成一组相机—LiDAR，在独立场景与队友Koide结果对照。
5. 完成AT128—QT128，再扩展环视，形成统一外参。

### TU Delft / ARS548

1. 隔离旧环境，原版示例先跑通。不要同时做ROS迁移、雷达模型修改和硬件适配。
2. 缺数据可用合成观测测试求解链路，但不宣称真实检测已通过。
3. 裸角反射器及加板对照测试，确认ARS548检测稳定且静态目标不被过滤。
4. 适配禾赛靶标检测，核查ring/扫描逻辑与孔对应，不以输出四个点作为成功标准。
5. 先做ARS548—一台LiDAR三维观测求解，处理参考点偏移与不同方向误差。
6. 分组结果可靠后，再考虑三模态联合优化；若无改善，不强行引入复杂度。

### FAST-Calib可替代的部分

如果最终购买了匹配FAST-Calib的四孔板，可优先跑其官方示例与一组相机—LiDAR实机标定，不强制先改成Fraunhofer靶标。双LiDAR和毫米波任务仍另行保留。

## 8. 交接后的第一轮检查

- 确认Linux发行版、ROS版本、CPU/GPU、可用驱动与现有工作区；未获实现任务前不自动升级系统或改服务器驱动。
- 确认各相机数量、有效尺寸/ROI、相机内参是否已采集完成。
- 确认偏振相机型号与图像输出格式。
- 确认板是否已购买：目前仅有商品示意图与PDF生成记录，不能认定实物已到货。
- 核查AT128/QT128的PointCloud2字段、强度有效性、校正文件与时间戳。
- 核查ARS548检测级输出、是否已做安装位姿补偿、是否保留静态目标。
- 与Koide队友统一内参、坐标约定、安装状态和独立验证集。
- 将环境复现、检测通过、外参求解、独立验证分别记录，不混成一个“跑通”。

## 9. 当前真正完成的事项

- 已完成方案讨论、官方资料检索与链接整理。
- 已生成两种尺寸普通棋盘PDF。
- 已生成两种尺寸ChArUco PDF及JSON，源图案识别检查通过。
- **没有在本轮对话实际安装/运行Fraunhofer、TU Delft、FAST-Calib或Koide。**
- **没有读取实机标定数据，没有产出相机内参或传感器外参矩阵。**
- 本文是交接上下文，不代表对Linux系统或远程服务器的任何修改授权。
