# Visual-Translationo

RoboMaster 装甲板视觉识别与追踪项目，基于 ROS 2、OpenCV 和 ONNX 模型实现从图像采集到目标识别、位姿解算、目标追踪和串口控制指令输出的完整链路。

## 项目功能

- 装甲板灯条检测与匹配
- 数字 ROI 提取与 ONNX 模型分类
- PnP 位姿解算
- 基于云台角度的世界坐标系转换
- 扩展卡尔曼滤波追踪目标
- 串口接收云台角度并发送瞄准指令
- 支持相机实时运行和离线视频测试
- 支持 Foxglove Bridge 可视化调试

## 项目结构

```text
.
├── DeepLearning/                         # 数字识别模型训练、测试和导出脚本
├── src/
│   ├── armor_plate_bringup/              # 一键启动 launch 文件
│   ├── armor_plate_identification/       # 图像采集、装甲板识别、数字分类、PnP
│   ├── armor_plate_interfaces/           # 自定义 ROS 2 消息
│   ├── armor_plate_serial/               # 串口通信节点
│   └── armor_plate_tracker/              # 目标选择、坐标转换、EKF 追踪
├── build_workspace.sh                    # 构建工作区并合并 compile_commands
└── merge_compile_commands.py             # 合并 clangd 使用的编译数据库
```

## 数据流

```text
Camera / Video
    |
    v
armor_plate_identification
    |  publish: /armor_plates
    v
armor_plate_tracker <----- /gimbal_angle ----- armor_plate_serial
    |  publish: /aim_command
    v
armor_plate_serial
    |
    v
Electrical control board
```

主要话题：

| Topic | 消息类型 | 说明 |
| --- | --- | --- |
| `/armor_plates` | `armor_plate_interfaces/msg/ArmorPlates` | 识别到的装甲板数组，包含位姿、数字和图像中心距离 |
| `/gimbal_angle` | `armor_plate_interfaces/msg/GimbalAngle` | 电控回传的云台 yaw/pitch |
| `/aim_command` | `armor_plate_interfaces/msg/AimCommand` | 视觉输出给电控的瞄准修正量 |
| `/tracker_debug` | `armor_plate_interfaces/msg/TrackerDebug` | tracker 调试数据 |
| `/tracker_data` | `armor_plate_interfaces/msg/TrackerData` | 测量值和滤波值 |
| `/filter_pose` | `geometry_msgs/msg/PoseStamped` | 滤波后的世界坐标系位姿 |
| `/measured_pose` | `geometry_msgs/msg/PoseStamped` | PnP 原始测量位姿 |

## 环境依赖

推荐环境：

- Ubuntu 22.04
- ROS 2 Humble
- OpenCV 4.x
- Eigen3
- cv_bridge、image_transport、camera_info_manager、tf2_ros
- MindVision 或 Galaxy 相机 SDK
- foxglove_bridge，可选，用于可视化

第三方相机 SDK 头文件和库位于：

```text
src/armor_plate_identification/third_parties/
```

## 构建

在 ROS 2 环境中执行：

```bash
source /opt/ros/humble/setup.bash
./build_workspace.sh
source install/setup.bash
```

也可以直接使用 `colcon`：

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
python3 merge_compile_commands.py
source install/setup.bash
```

## 运行

实时相机模式：

```bash
ros2 launch armor_plate_bringup run.launch.py
```

离线视频测试：

```bash
ros2 launch armor_plate_bringup test.launch.py video_path:=/path/to/video.mp4
```

如果不需要启动 Foxglove Bridge：

```bash
ros2 launch armor_plate_bringup run.launch.py use_foxglove:=false
ros2 launch armor_plate_bringup test.launch.py use_foxglove:=false
```

单独启动节点：

```bash
ros2 launch armor_plate_identification run.launch.py
ros2 launch armor_plate_identification test.launch.py video_path:=/path/to/video.mp4
ros2 launch armor_plate_tracker run.launch.py
ros2 launch armor_plate_serial run.launch.py
```

## 参数配置

识别参数：

```text
src/armor_plate_identification/config/params.yaml
```

常用参数：

| 参数 | 说明 |
| --- | --- |
| `target_color` | 目标颜色，`BLUE` 或 `RED` |
| `camera_type` | 相机类型，`mindvision` 或 `galaxy` |
| `exposure_time` | 曝光时间 |
| `gain` | 相机增益 |
| `model_path` | 数字分类 ONNX 模型路径 |
| `label_path` | 数字标签文件路径 |
| `number_threshold` | 数字分类置信度阈值 |
| `ignore_labels` | 需要过滤的分类标签 |
| `debug_base` | 基础调试开关 |
| `debug_identification` | 识别结果绘制开关 |
| `debug_preprocessing` | 预处理图像显示开关 |
| `debug_number_classification` | 数字分类显示开关 |

灯条匹配参数：

| 参数 | 说明 |
| --- | --- |
| `max_angle_diff` | 左右灯条最大角度差 |
| `min_length_ratio` | 左右灯条最小长度比 |
| `min_x_diff_ratio` | 局部 x 方向最小间距比例 |
| `max_y_diff_ratio` | 局部 y 方向最大偏移比例 |
| `min_distance_ratio` | 最小距离比例 |
| `max_distance_ratio` | 最大距离比例 |

追踪参数：

```text
src/armor_plate_tracker/config/params.yaml
```

| 参数 | 说明 |
| --- | --- |
| `max_lost_time` | 目标丢失多久后重置追踪器 |
| `mutation_yaw_threshold` | 装甲板 yaw 突变检测阈值，单位 rad |
| `debug` | tracker 调试输出开关 |

串口参数：

```text
src/armor_plate_serial/config/params.yaml
```

| 参数 | 说明 |
| --- | --- |
| `device_name` | 串口设备名，例如 `/dev/ttyACM0` |
| `baud_rate` | 波特率 |

## 数字识别模型

模型文件默认使用：

```text
src/armor_plate_identification/model/number_cnn.onnx
src/armor_plate_identification/model/label_cnn.txt
```

训练和测试脚本位于 `DeepLearning/`：

```bash
cd DeepLearning
python train.py
python batch_inference.py
```

训练完成后，将导出的 `number_cnn.onnx` 放到 `src/armor_plate_identification/model/`，并确认 `params.yaml` 中的 `model_path` 和 `label_path` 与文件名一致。

## 调试建议

1. 先用 `test.launch.py` 跑离线视频，确认识别、分类和 tracker 输出正常。
2. 再切到 `run.launch.py` 连接真实相机，调整曝光、增益和目标颜色。
3. 如果没有串口设备，可以先单独运行识别和 tracker，或在 bringup 中临时关闭串口节点。
4. 使用 Foxglove 连接 `ws://localhost:8765` 查看话题和位姿数据。

## 常见问题

### 找不到相机 SDK 动态库

确认 `third_parties` 中包含对应 SDK 的 `include` 和 `lib/amd64`，并重新 `source install/setup.bash`。

### 串口无法打开

检查设备名和权限：

```bash
ls /dev/ttyACM*
sudo usermod -aG dialout $USER
```

修改用户组后需要重新登录。

### clangd 找不到头文件

构建后运行：

```bash
python3 merge_compile_commands.py
```

然后确认根目录存在：

```text
build/compile_commands.json
```
