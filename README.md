# Hikvision–Livox 装甲板检测与测距

ROS2 Humble 工作空间：海康相机提供图像，OpenVINO 模型输出装甲板检测框，Livox Mid-70 提供点云，将单帧点云投影到图像后计算框内三维距离。

本文面向第一次接手本仓库的开发者，说明当前实际运行的代码、接口、配置和开发入口。最后核对：2026-10-19。

**当前默认链路是 C++ 单帧处理。** `hik_livox_system` 不再只是启动包，其中的 `stamper_cpp`、`projector_cpp` 是生产链路入口。`livox_pcd_projection` 仍提供配置、标定文件和保留的 Python 实现，但统一启动不会执行 Python 投影节点。

## 1. 快速运行

当前部署环境：Ubuntu 22.04、ROS2 Humble、海康 MVS SDK、Livox SDK、OpenVINO、OpenCV、Foxglove Bridge。当前电脑已配置这些依赖；本仓库包含机器相关路径，不是新系统上的一键安装工程。

```bash
cd /home/szx/works/hik_livox_workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hik_livox_system bringup.launch.py open_foxglove:=false
```

`open_foxglove:=false` 仅取消自动打开 Studio，Bridge 仍会启动。省略该参数则尝试自动打开并连接 Foxglove。前台启动用 `Ctrl+C` 停止整套节点，避免重复启动抢占相机和雷达。

| launch 参数 | 默认值 | 作用 |
|---|---|---|
| `my_color` | `2` | 我方颜色：0=我方红/保留蓝，1=我方蓝/保留红，2=红蓝都保留 |
| `publish_debug` | `true` | 检测器是否发布 `/armor_debug` |
| `projection_debug` | `true` | 投影器是否发布 `/debug_image` |
| `open_foxglove` | `true` | 是否自动打开 Studio |
| `foxglove_port` | `8765` | Bridge 端口 |

例如关闭可视化图像、保留检测与测距：

```bash
ros2 launch hik_livox_system bringup.launch.py \
  publish_debug:=false projection_debug:=false open_foxglove:=false
```

目前 Bridge 绑定 `127.0.0.1`，本机 Foxglove 连接 `ws://localhost:8765`。推荐 Image 面板查看 `/armor_debug`、`/debug_image`，Raw Messages 查看 `/distances`。Livox 输出为 `CustomMsg`，不能直接当作标准 `PointCloud2` 输入 3D 面板。

## 2. 架构与代码入口

```text
hik_camera_node
  └─ /image_raw ──┬─ armor_detector_node ── /armor_detections ──┐
                  │                   └── /armor_debug        │
                  └──────────────────────────────────────────┤
                                                             ▼
livox_ros2_driver_node                                  projector_cpp
  └─ /livox/lidar → stamper_cpp → /livox/stamped ──────────────┤
                                                             ├─ /distances
                                                             └─ /debug_image

foxglove_bridge：订阅 ROS 话题，经 WebSocket 提供给 Studio
```

### 推荐阅读顺序

| 顺序 | 文件 | 需要理解的内容 |
|---|---|---|
| 1 | `src/hik_livox_system/launch/bringup.launch.py` | 节点组成、参数来源、设备配置、话题连接 |
| 2 | `src/hik_livox_system/src/projector.cpp` | 核心测距算法、缓存、时序判定、调试图 |
| 3 | `src/ros2_armor_detector/src/armor_detector_node.cpp` | 模型接入、角点转 bbox、检测消息和时间戳 |
| 4 | `src/hik_livox_system/src/stamper.cpp` | 点云收到时刻转为 ROS header 时间 |
| 5 | `src/NNdeployment/interface/network_deployment_interface.hpp` | `ArmorModel`、`JsonConfig` 等推理接口 |
| 6 | `src/hik_livox_system/test/test_native_pipeline.py` | 可执行的接口示例和边界行为 |

### 模块职责

| 包 / 目录 | 职责 |
|---|---|
| `hik_livox_system` | 统一启动、当前 C++ 转发与投影程序、原生链路回归测试 |
| `ros2_hik_camera`，ROS 包名为 `hik_camera` | 相机 SDK 接入、图像与 CameraInfo 发布 |
| `ros2_armor_detector` | ROS 图像与模型接口适配，发布检测与检测调试图 |
| `NNdeployment` | 模型预处理、OpenVINO 推理与后处理；检测包通过 CMake 子目录编译该实现 |
| `livox_ros2_driver` | 雷达设备接入与点云发布 |
| `livox_sdk_vendor` | SDK 构建和本机 IP 选择补丁 |
| `livox_pcd_projection` | 当前仍使用的配置和标定文件；保留 Python 版本用于比较/测试 |
| `livox_interfaces` / `dist_msg` | 自定义点云与距离消息 |
| `models/openvino` | 当前模型的 XML/BIN |

## 3. ROS 接口与消息语义

| 话题 | 类型 | 发布者 | 说明 |
|---|---|---|---|
| `/image_raw` | `sensor_msgs/msg/Image` | `/hik_camera` | 当前 1440×1080、RGB8 |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | `/hik_camera` | 相机标定信息；当前投影器直接读内参文件，不订阅它 |
| `/armor_detections` | `vision_msgs/msg/Detection2DArray` | `/armor_detector` | 源图像像素坐标中的轴对齐 bbox |
| `/armor_debug` | `sensor_msgs/msg/Image` | `/armor_detector` | BGR8，模型角点、类别与置信度 |
| `/livox/lidar` | `livox_interfaces/msg/CustomMsg` | `/livox_lidar_publisher` | 原始雷达点云，目标 10 Hz |
| `/livox/stamped` | 同上 | `/stamper` | 点不变，仅更新 ROS header 时间 |
| `/distances` | `dist_msg/msg/Dist` | `/projector` | 每个点云回调发布一次，包含空结果 |
| `/debug_image` | `sensor_msgs/msg/Image` | `/projector` | RGB8，默认 720×540 的点云投影调试图，不叠加检测框 |

### 检测消息

检测器将模型角点取轴对齐外接矩形，写入：

```text
bbox.center.position.x / y
bbox.size_x / size_y
bbox.center.theta = 0
results[0].hypothesis.class_id / score
```

数组与各 detection 的 header 沿用输入图像 header。无目标时仍发布空 `detections` 数组。`detection.id` 当前写类别名，**不是跨帧跟踪 ID**；当前链路没有目标跟踪。投影器使用 bbox 与第一个 hypothesis，未使用旋转角或进一步按置信度过滤。

### 距离消息

定义：`src/dist_msg/msg/Dist.msg`。

```text
std_msgs/Header header
uint8 count
string[] obj_classes
float64[] distances
```

- `obj_classes[i]` 对应 `distances[i]`，单位为米，数组顺序来自当前使用的检测数组。
- header 沿用本次 `/livox/stamped` 的 header，时间是转发节点收到点云的时刻。
- 有效检测框中没有点：保留该类别，距离为 `NaN`。
- 检测缺失、过期或空数组：发布 `count=0`、空数组。
- 没有 hypothesis 的 detection 被跳过；受 `uint8 count` 限制，最多发布 255 条。
- 消息没有检测源时间戳、匹配 ID、命中点数或有效性枚举。若需要关联、区分空结果原因，需扩展消息；目前相关信息只在日志中提供。

## 4. 单帧投影与测距算法

实现位置：`Projector::process()`，每收到一条 `/livox/stamped` 触发一次。没有点云历史缓存。

### 4.1 投影

遍历 `cloud->points` 的全部实际元素，对每点 `p_lidar=(x,y,z)`：

```text
p_camera_h = T_camera_from_lidar × [x, y, z, 1]^T
q          = K × p_camera
u, v       = q.x / q.z, q.y / q.z
range      = sqrt(x² + y² + z²)
```

过滤非有限坐标、相机深度 `z_camera <= 1e-6`、非有限 UV，以及图像范围外点。有效点保存为 `{range, u, v}`，后续测距和绘图复用该数组。

**当前为针孔投影，不应用畸变系数。** `range` 是从雷达原点计算的三维欧氏距离，不是相机 Z 深度，也不是雷达 X 方向距离。

### 4.2 框内选点与中值

对每个有效检测框：

```text
half_w = bbox.size_x × bbox_size / 2
half_h = bbox.size_y × bbox_size / 2

inside = center_x-half_w <= u <= center_x+half_w
      && center_y-half_h <= v <= center_y+half_h

distance = median(range[inside])
```

`bbox_size=0.8` 表示取中心 80% 宽、80% 高的区域，减少框边背景干扰；不是取 80% 数量的点。需要完整 bbox 时设为 `1.0`。

所有满足条件的点都进入中值计算。实现用 `std::nth_element`，偶数个点取两个中间值的平均值。重叠框可重复使用同一个点，各框独立计算。没有深度聚类、遮挡判断或背景分离，因此 bbox 内背景点占多数时，中值也可能是背景距离。

计算量约为投影 O(N)、框内扫描 O(B×M)，其中 N 为输入点数，M 为投影有效点数，B 为框数。当前约 1 万点/帧、少量框，先一次投影再按 UV 筛选即可；没有额外的空间索引。

### 4.3 可视化分支

距离发布后才生成调试图：选图 → 缩放 → 对投影有效点按步长采样 → 按距离着色 → 写像素并发布。

- 默认 `debug_max_points=5000`，只限制画点数；`0` 表示绘制全部投影有效点。
- 默认 `debug_scale=0.5`，1440×1080 缩为 720×540；测距 UV 坐标仍为原图坐标。
- 默认 `debug_rate=10`；使用点云时间戳做限频，阈值为 `0.9/debug_rate`，为输入抖动留 10% 容差，是软限频。
- 同像素多点会覆盖，不使用深度缓冲。
- 关闭 debug 不影响距离计算，也不再订阅相机图像。

**这是同一回调中的两个分支，不是两个独立线程。** 距离不依赖画图采样，但画图仍消耗当前回调时间；显示过慢会影响后续帧处理。

## 5. 缓存、时间戳与执行模型

三个 C++ 节点当前均通过 `rclcpp::spin()` 使用单线程执行器。投影器没有三路 `ApproximateTimeSynchronizer`，而是缓存检测与图像，由点云触发计算。

| 输入 | QoS / 缓存 | 用途 |
|---|---|---|
| 检测 | 默认 reliable，深度 1；保存最新一条 | 复用最近检测框 |
| 点云 | 默认 reliable，深度 10 | 每个回调处理一条消息 |
| 图像，仅 debug | SensorDataQoS、订阅深度 2；另有最多 20 张图像的 deque | 选择调试图背景 |

检测有效条件：

```text
abs(cloud.header.stamp - detections.header.stamp) <= max_data_age
```

统一启动下 `max_data_age=0.5s`。这不是检测“到达节点多久”的判断，比较的是检测**源图像时间**与点云的 ROS header 时间；采用绝对差，也允许阈值内检测源时间晚于点云时间。

调试图优先找最接近有效检测源时间的缓存图像；没有有效检测则以点云时间为目标。若候选图像距点云超过 `max_image_age=0.2s`，退回最新图像；最新图像仍超限则不发调试图。检测缺失/过期不会直接阻止图像发布。

需要理解的边界：

- Stamper 仅用 `now()` 重写 ROS header，保留点、`timebase` 和点偏移时间；没有把雷达硬件时钟标定到相机时钟，也未使用逐点时间做补偿。
- 10 Hz 测距可以复用较低频率的检测框；它不等价于 10 Hz 的独立检测，也不保证图像与点云为同一采样时刻。
- “全点测距”指每个被处理消息的所有符合条件点，没有空间降采样；不承诺过载时任何时刻的消息都不会因有限队列丢失。
- 当前没有运动补偿、跟踪、并行处理或按检测源时间反查历史点云。快速运动场景需重新设计时间匹配，不能只扩大过期阈值。

## 6. 配置入口与优先级

统一 launch 从包 share 目录加载 `livox_pcd_projection/config/project.yaml`，再覆盖部分节点参数。优先级为：

```text
C++ declare_parameter 默认值 < project.yaml < launch 后置参数字典
```

因此 YAML 中的 `debug=False`、`max_data_age=0.2`、`debug_max_points=10000`，在**统一启动**下实际分别是 true、0.5、5000。

### 投影节点参数

| 参数 | 统一启动值 | 来源 / 意义 |
|---|---:|---|
| `camera_topic` | `/image_raw` | YAML；debug 输入 |
| `detection_topic` | `/armor_detections` | YAML |
| `lidar_topic` | `/livox/stamped` | YAML |
| `intrinsic_path` / `extrinsic_path` | 当前工作空间中的绝对路径 | YAML；换机器必须检查 |
| `image_width` / `image_height` | 1440 / 1080 | YAML；原始投影边界，不能改为缩放图大小 |
| `bbox_size` | 0.8 | YAML；框内取点范围 |
| `debug` | true | launch；由 `projection_debug` 控制 |
| `max_data_age` | 0.5 | launch；检测匹配阈值，秒 |
| `max_image_age` | 0.2 | C++ 默认；调试背景图匹配阈值，秒 |
| `debug_max_points` | 5000 | launch；仅影响显示 |
| `debug_scale` | 0.5 | launch；仅影响显示 |
| `debug_rate` | 10.0 | launch；仅影响显示 |

旧 YAML 中的 `lidar_threshold`、`refresh_rate` 不被当前 C++ 实现使用。C++ 的点云处理频率由输入驱动。

除第 1 节列出的 launch 参数外，其他节点参数没有自动开放为 `ros2 launch ... 参数:=值`；需要改对应 YAML / launch 字典，或单独运行节点时传 `--ros-args -p`。

参数在构造时读入成员变量，没有动态更新回调。**修改后需重启节点**；`ros2 param set` 成功不代表处理逻辑已经应用新值。用以下命令确认节点启动值：

```bash
ros2 param get /projector max_data_age
ros2 param get /projector debug_max_points
ros2 param get /armor_detector cpu_threads
```

### 模型与线程配置

模型配置：`src/ros2_armor_detector/config/armor_v8_cpu.json`。

```json
{
  "armor_v8": {
    "xml": "Infantry-v8n-fp16-20260726-D1.8w-B16/Infantry-v8n-fp16-20260726-D1.8w-B16.xml",
    "infer_mode": "sync",
    "deploy_way": "openvino",
    "postprocess_mode": "auto",
    "device": "CPU",
    "score_threshold": 0.5
  }
}
```

模型路径由 `model_root` 与 JSON 的 `xml` 拼接；默认根目录在 `models/openvino`，当前输入为 640×480。ROS 适配层将相机 RGB 转成模型 BGR 输入，并由推理库完成缩放、归一化等操作。

检测节点 `cpu_threads=4`，写入进程环境 `NNDEPLOYMENT_CPU_THREADS`，由 `network_deployment_openvino.cpp` 在 CPU 后端读取；正数时设置线程数、单 stream 和 ANY_CORE。`0` 保留 OpenVINO 默认配置。检测节点会覆盖同名环境变量，应通过节点参数配置。OpenCV 线程数设为 1，图像订阅队列深度为 1，优先处理最新图像。

当前使用同步推理，消息 header 对应本次输入图像。若迁移异步/流水线推理，必须一起检查输出对应图像与 header 的关联，不能只改 JSON 中的 `infer_mode`。

## 7. 标定与硬件配置

### 相机与外参

| 配置 | 文件 |
|---|---|
| 相机曝光、增益 | `src/ros2_hik_camera/config/camera_params.yaml` |
| 相机 CameraInfo | `src/ros2_hik_camera/config/camera_info.yaml` |
| 投影使用的 K | `src/livox_pcd_projection/config/intrinsic.txt` |
| 投影使用的 T | `src/livox_pcd_projection/config/extrinsic.txt` |

当前相机 1440×1080，`fx=1802.056498`、`fy=1802.662989`、`cx=704.656747`、`cy=559.419082`，历史标定 RMS 约 0.400799 px。稳定相机设置为 `exposure_time=10000.0`、`gain=16.0`，曾有其他增益值导致驱动退出。

投影的 `T` 是 **雷达 → 相机** 的 4×4 齐次变换，已从历史标定结果转换为所需方向；不要因为文件名或标定工具中的变量名再求一次逆。加载器读取内参文件前 9 个、外参文件前 16 个数值，跳过标题行；内参文件后面的畸变系数没有参与当前投影。

移动相机/雷达相对安装姿态必须重新标定；改变相机分辨率、ROI 或图像几何时需同步检查 K 和原图边界。历史工具在 `/home/szx/works/direct_visual_lidar_calibration`，结果在 `/home/szx/works/calibration_preprocessed/calib.json`。

### Livox 与网络

驱动配置：`src/livox_ros2_driver/config/livox_lidar_config.json`。当前设备广播码 `3GGDMC600200361`，雷达 IP `192.168.1.136`，有线主机 IP `192.168.1.50/24`。

```bash
ip -br addr
ip route get 192.168.1.136
```

路由应走有线网卡并使用 `src 192.168.1.50`；网卡曾为 `enxdc045ac808ee`，实际以机器输出为准。保留 WiFi，远程连接依赖它。

SDK 曾错误选择同网段 WiFi 地址，当前通过 `livox_sdk_vendor/patches/route-local-ip.patch` 和相应 CMake 补丁脚本按实际路由选择本机地址。

统一启动固定 `xfer_format=1`（CustomMsg）、`publish_freq=10.0`、`frame_id=livox_frame`。标定工具使用 PointCloud2 是另一条工作流；直接切换驱动格式会使当前投影器订阅类型不匹配。

## 8. 构建与开发定位

### 构建

首次构建整套工作空间，需先在系统配置好 MVS、OpenVINO、Livox SDK 相关依赖：

```bash
cd /home/szx/works/hik_livox_workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

已有依赖构建结果时，按修改范围增量构建：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select hik_livox_system ros2_armor_detector livox_pcd_projection \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

生产 C++ 可执行程序安装在 `install/hik_livox_system/lib/hik_livox_system/`。编译不会替换运行中的进程；更新可执行程序或共享库后需重启相关节点。构建输出在 `build/`、`install/`、`log/`，运行日志通常在启动终端与 `~/.ros/log/`。

### 按需求找代码

| 修改目的 | 修改入口 | 增量构建包 |
|---|---|---|
| 改节点组成、默认配置、话题连接 | `hik_livox_system/launch/bringup.launch.py` | `hik_livox_system` |
| 改取点范围、中值、过滤、时间策略、调试图 | `hik_livox_system/src/projector.cpp` | `hik_livox_system` |
| 改点云时间戳转换 | `hik_livox_system/src/stamper.cpp` | `hik_livox_system` |
| 改 bbox 输出、检测消息、颜色筛选入口、图像队列 | `ros2_armor_detector/src/armor_detector_node.cpp` | `ros2_armor_detector` |
| 改预处理/推理/后处理 | `NNdeployment/src/`、`NNdeployment/interface/` | `ros2_armor_detector`（其内部编译该库） |
| 换模型、阈值或推理设备 | `ros2_armor_detector/config/armor_v8_cpu.json` 与模型目录 | `ros2_armor_detector` |
| 改内外参、共用配置 | `livox_pcd_projection/config/` | `livox_pcd_projection` |
| 扩展距离接口 | `dist_msg/msg/Dist.msg`，以及发布端/消费端 | 重建 `dist_msg` 及其所有下游 |

表中路径均相对于 `src/`。旧 `livox_pcd_projection/livox_pcd_projection/projectCloud.py`、`timestamp.py` 不是当前统一启动入口；改它们不会改变 C++ 生产链路。

不要并行启动旧工作空间 `/home/szx/works/1`、`mid70_driver`、`livox_projection` 中的设备驱动，也不要在 `/opt/ros/humble` 中直接修改系统包。

## 9. 回归测试

硬件无关测试依赖已编译的 C++ 二进制、ROS 消息和 Python pytest，使用独立 ROS 域，不向通常默认域中的实机话题注入测试数据：

```bash
cd /home/szx/works/hik_livox_workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=89 PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  src/hik_livox_system/test/test_native_pipeline.py \
  src/livox_pcd_projection/test/test_projectCloud.py
```

当前预期为 `3 passed`：

| 测试文件 | 覆盖内容 |
|---|---|
| `test_native_pipeline.py` | 启动实际 C++ 转发/投影进程；点与 timebase 保留、header 更新、单帧独立、奇偶中值、无效/零深度点过滤、空点云、显示抽样/缩放、过期检测时图像继续输出 |
| `test_projectCloud.py` | Python 保留实现的合成点投影、框内中值、无效点过滤、绘图抽样不改变距离输入 |

如实机已经使用域 89，测试换一个空闲域。测试通过不代表标定、模型精度、网络或运动时序通过实机验证。目前这些 pytest 通过上述命令直接执行，未注册为 CMake/colcon 的自动测试目标。

## 10. 运行观测与故障定位

### 日志

投影节点约每 5 秒输出：

```text
points=9984 valid=4200 boxes=1 hits=120 detection_age=0.120s processing=5.00ms
```

- `points`：消息实际点数；`valid`：投影过滤后点数。
- `boxes`：距离条目数；`hits`：各框命中点数之和，重叠框会重复计数。
- `detection_age`：点云时间减检测源图像时间；无检测时为 -1，负值也可能表示检测源时间晚于点云时间。
- `processing`：点云回调耗时，包含当次实际执行的绘图，不是端到端延迟。

检测日志 `image conversion + model` 包含颜色转换、预处理、推理、后处理，未包含后续 debug 绘制与完整传输等待。

### 常用检查

```bash
ros2 node list
ros2 topic info /livox/lidar
ros2 topic info /livox/stamped
ros2 topic hz /armor_detections
ros2 topic hz /debug_image
ros2 topic hz /distances
ros2 topic echo /distances --once
```

频率检查建议逐项运行。Python `ros2 topic hz/echo` 对包含大量嵌套点的 CustomMsg 有明显解析开销，同时开多个观察工具会影响被测系统；发布者存在也不代表有持续数据。

| 现象 | 优先检查 |
|---|---|
| `/livox/lidar` 无数据 | 电源、网线、有线 IP/路由、驱动日志、广播码配置 |
| 原始雷达正常，stamped 低频 | 是否仍运行旧 Python stamper、是否有重复发布者 |
| 图像正常，检测低频 | 模型耗时、CPU 负载、`cpu_threads`、图像订阅队列 |
| 距离 `count=0` | 检测是否空、是否超出 `max_data_age`，不是直接等同雷达故障 |
| 有框但距离 NaN / hits=0 | 目标是否被扫描、框是否过小、内外参是否对齐 |
| 距离跳变或落在背景 | bbox 背景混点、旧检测框复用、相对运动、标定偏差 |
| ROS 调试图有频率，Foxglove 卡 | WebSocket 连接、图像面板数量、Studio CPU/渲染负载 |
| 参数修改无效 | launch 覆盖 YAML、未加载新 install、节点未重启 |

2026-10-19 本机实测：原始雷达约 10 Hz；重构后 stamped、距离、融合图约 10 Hz，检测约 9～11 Hz。重构前 Python stamped 约 4.4 Hz、检测约 4.9 Hz，投影又受过严时间门限影响几乎不输出。这些是当前设备和负载下的观测值，不是跨机器性能保证。

## 11. 后续开发需注意的约束

- 模型角点已经在 ROS 适配层压缩成轴对齐 bbox；若要用四边形或装甲板姿态，需要扩展接口，不能从当前 bbox 还原。
- 测距没有目标深度聚类，也不输出点数或不确定度；距离中值不是自动识别目标前景的保证。
- 收到时刻重打时间戳解决了时间基准可比较的问题，但没有消除采集/网络延迟；动态场景优化应优先建立正确的采样时间关联。
- 图像 deque 仅为可视化选图，点云仍是单帧。若实现历史匹配，需明确窗口、内存和延迟策略，不要恢复无界累计。
- 多线程化时必须保护检测缓存与图像 deque，当前线程安全建立在单线程执行器上。
- `debug_rate` 是基于点云时间的软限频，不是独立定时器；改高它不能超过输入频率。
- 跨机器部署需清理 `model_root`、标定路径等硬编码，并验证相机 SDK、网络、标定、CPU 线程配置和 ROS 消息版本。
