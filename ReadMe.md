# RoboMaster 装甲板视觉识别系统

基于 **ROS 2 Humble** + **OpenCV 4.x** + **ONNX Runtime** 的装甲板识别、跟踪与瞄准解算方案。

---

## 项目简介

完整的视觉处理链路：图像采集 → 预处理 → 灯条检测与配对 → 数字识别 → PnP 位姿解算 → 世界坐标系卡尔曼滤波跟踪 → 串口通信。

支持**相机实时运行**与**离线视频调试**两种模式。

---

## 数据流

```
                              [相机/视频]
                                   │
                                   ▼
                    ┌──────────────────────────────┐
                    │  armor_plate_identification  │
                    │    (识别 + PnP + 数字识别)    │
                    └──────────────────────────────┘
                                   │
                                   │ ArmorPlates
                                   ▼
                    ┌──────────────────────────────┐
                    │     armor_plate_tracker      │
                    │   (世界坐标系KF + 时间对齐)   │
                    │                              │
                    │  订阅 /gimbal_angle ←─────────┼──── [电控回传]
                    │       (GimbalAngle)           │      │
                    └──────────────────────────────┘      │
                                   │                      │
              ┌────────────────────┼────────────────────┐  │
              │                    │                    │  │
              ▼                    ▼                    ▼  │
   ┌─────────────────┐  ┌─────────────────────┐  ┌────────┴─────────┐
   │  armor_plate_   │  │ data_visualization_ │  │      RViz        │
   │     serial      │  │       node          │  │                  │
   │  (串口发送)      │  │  (OpenCV 窗口)      │  │  /filter_pose    │
   │                 │  │                     │  │  /measured_pose  │
   │ AimCommand      │  │   TrackerDebug      │  │  Marker / TF     │
   └─────────────────┘  └─────────────────────┘  └──────────────────┘
              │
              ▼
           [电控]
```

**消息说明**

| Topic | 类型 | 说明 |
|-------|------|------|
| `/armor_plates` | `ArmorPlates` | 检测到的装甲板数组（含位姿、数字、图像中心距） |
| `/gimbal_angle` | `GimbalAngle` | 电控回传的绝对 yaw/pitch（带时间戳） |
| `/aim_command` | `AimCommand` | 控制指令（delta_yaw, delta_pitch，单位 rad） |
| `/tracker_debug` | `TrackerDebug` | 相机系下测量点与滤波点（用于图像叠加绘制） |
| `/tracker_data` | `TrackerData` | measurement/filter 的 yaw/pitch |
| `/filter_pose` | `PoseStamped` | 滤波后世界坐标系位姿 |
| `/measured_pose` | `PoseStamped` | PnP 原始测量世界坐标系位姿 |

---

## 功能包说明

| 功能包 | 职责 | 节点 | 订阅 | 发布 |
|--------|------|------|------|------|
| `armor_plate_identification` | 图像采集、预处理、灯条检测、PnP、数字识别 | `ArmorPlateIdentifcation` (相机) / `Test` (视频) | — | `/armor_plates`, TF |
| `armor_plate_tracker` | 时间对齐、目标选择、世界坐标系 KF | `armor_plate_tracker_node` | `/armor_plates`, `/gimbal_angle` | `/aim_command`, `/tracker_debug`, `/tracker_data`, `/filter_pose`, `/measured_pose`, TF |
| `armor_plate_data_visualization` | yaw/pitch 实时曲线绘制 | `data_visualization_node` | `/tracker_debug` | (OpenCV 窗口) |
| `armor_plate_serial` | 串口双向通信 | `serial_node` | `/aim_command` | `/gimbal_angle`, (串口) |
| `armor_plate_interfaces` | 自定义消息定义 | — | — | — |
| `armor_plate_bringup` | 一键启动组合 | `run.launch.py` / `test.launch.py` | — | — |

---

## 技术特点

### 1. 预处理

- 灰度转换 → 固定阈值二值化 (`threshold=160`) → `3×3` 矩形核膨胀
- 不依赖颜色通道，过曝/杂光场景更鲁棒
- 支持 `target_color` 参数切换红/蓝方（用于数字识别与后续逻辑）

### 2. 灯条检测与匹配

- 轮廓筛选：面积 + 长宽比约束
- 直线提取：`fitEllipse` 方向 + `minAreaRect` 长度约束
- 几何约束配对：6 个参数（角度差、长度比、中心距比、Y/X 差比等）
- 打分 + NMS：按平行度、长度相似度排序，一个灯条只归属一个装甲板

### 3. 数字识别

- 装甲板中心 ROI 透视变换提取
- **ONNX Runtime MLP** 推理，输出 0-9 + negative
- 模型文件：`model/number_cnn.onnx`
- 训练工具链见 `DeepLearning/`

### 4. PnP 位姿解算

- `cv::solvePnP` 解算 `tvec` + 四元数
- 动态读取 `CameraInfo` 获取相机内参与畸变系数
- yaw/pitch 在 Tracker 内从相机系转换：
  ```cpp
  yaw   = -atan2(tx, tz);
  pitch = atan2(-ty, sqrt(tx*tx + tz*tz));
  ```

### 5. 目标跟踪（世界坐标系 KF）

- **时间对齐**：`ArmorPlates.header.stamp` = 图像到手时刻；维护 `deque` 缓存最近 50 帧 `/gimbal_angle`，最近邻匹配
- **旋转矩阵**：`R_w←c = Rx(pitch)·Ry(-yaw)`，与四元数严格对应
- **x/y/z 三个独立 3 阶 KF**（位置-速度-加速度），在世界坐标系下 predict/correct
- 目标选择：未初始化时选图像中心最近；已初始化时选世界系下与预测位置最近
- 突变检测（装甲板姿态 yaw 突变 > 阈值时重置 KF）
- 丢失处理（`max_lost_time=0.5s` 超时重置）
- KF 参数：`Q=diag(0.001,0.001,0.001)`, `R=0.01`

### 6. 串口双向通信

**视觉 → 电控** (`0xA5 0x5A`)：
```c
typedef struct {
    uint8_t  sof1;              // 0xA5
    uint8_t  sof2;              // 0x5A
    uint8_t  seq;
    uint8_t  target_valid;
    int16_t  delta_yaw_1e4rad;
    int16_t  delta_pitch_1e4rad;
    uint16_t crc16;             // CRC16/MODBUS，前8字节
} VisionToEcFrame_t;
```

**电控 → 视觉** (`0x5A 0xA5`)：
```c
typedef struct {
    uint8_t  sof1;              // 0x5A
    uint8_t  sof2;              // 0xA5
    uint8_t  seq_echo;
    int32_t  yaw_actual_1e4rad;
    int32_t  pitch_actual_1e4rad;
    uint16_t crc16;             // CRC16/MODBUS，前11字节
} EcToVisionFrame_t;
```

- 发送：订阅 `/aim_command` 即时发送，`int16_t = rad * 10000.0f`
- 接收：独立线程 + 状态机解析，解析成功后填充 `stamp = now()` 发布 `/gimbal_angle`
- 无目标时自动停发

---

## 快速开始

### 环境依赖

- Ubuntu 22.04
- ROS 2 Humble
- OpenCV 4.x
- Eigen3
- ONNX Runtime
- MindVision / Galaxy 相机 SDK（已包含在 `third_parties/`）

### 编译

```bash
cd /home/minzhi/ws05_fourth_assessment

colcon build --packages-select \
  armor_plate_interfaces \
  armor_plate_identification \
  armor_plate_tracker \
  armor_plate_data_visualization \
  armor_plate_serial \
  armor_plate_bringup

source install/setup.bash
```

### 运行

#### 一键启动（推荐）

```bash
# 相机实时全链路
ros2 launch armor_plate_bringup run.launch.py

# 视频回放测试
ros2 launch armor_plate_bringup test.launch.py video_path:=/path/to/video.mp4
```

#### 单独启动

```bash
# 主程序（需连接相机）
ros2 launch armor_plate_identification run.launch.py

# 离线测试（视频文件）
ros2 launch armor_plate_identification test.launch.py video_path:=/path/to/video.mp4

# Tracker / 可视化 / 串口
ros2 launch armor_plate_tracker run.launch.py
ros2 run armor_plate_data_visualization data_visualization_node
ros2 run armor_plate_serial serial_node
```

---

## 参数配置

全局参数集中在 `armor_plate_identification/config/params.yaml`：

| 参数 | 类型 | 说明 |
|------|------|------|
| `target_color` | string | `"RED"` / `"BLUE"` |
| `camera_type` | string | `"mindvision"` / `"galaxy"` |
| `exposure_time` | float | 曝光时间（μs） |
| `gain` | float | 增益 |
| `debug_base` | bool | 基础调试（图像显示、键盘监听） |
| `debug_identification` | bool | 绘制检测框与参数 |
| `debug_preprocessing` | bool | 显示预处理中间结果 |
| `debug_number_classification` | bool | 显示数字识别结果 |

**灯条匹配参数**（支持运行时键盘实时调节 1-6 键 + T/G）：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `max_angle_diff` | 10.0° | 最大角度差 |
| `max_y_diff_ratio` | 1.0 | 最大 Y 方向高度差与灯条长度比 |
| `min_distance_ratio` | 0.1 | 最小中心距与灯条长度比 |
| `max_distance_ratio` | 0.8 | 最大中心距与灯条长度比 |
| `min_length_ratio` | 0.7 | 最小长度比（短/长） |
| `min_x_diff_ratio` | 0.75 | 最小 X 方向间距与灯条长度比 |

Tracker 参数在 `armor_plate_tracker/config/params.yaml`：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_lost_time` | 0.5 | 丢失超时（秒） |
| `mutation_yaw_threshold` | 3.0 | 突变检测阈值（度） |

---

## 目录结构

```
ws05_fourth_assessment/
├── src/
│   ├── armor_plate_bringup/           # 一键启动
│   ├── armor_plate_identification/    # 识别主包（相机/视频 + PnP + 数字识别）
│   │   ├── config/params.yaml
│   │   ├── launch/run.launch.py
│   │   ├── launch/test.launch.py
│   │   ├── src/
│   │   ├── model/number_cnn.onnx      # ONNX 数字识别模型
│   │   └── third_parties/             # MindVision / Galaxy SDK
│   ├── armor_plate_tracker/           # 跟踪 + 世界坐标系 KF
│   │   ├── config/params.yaml
│   │   └── launch/run.launch.py
│   ├── armor_plate_data_visualization/# 数据可视化
│   ├── armor_plate_serial/            # 串口双向通信
│   └── armor_plate_interfaces/        # 自定义消息
├── DeepLearning/                      # 数字识别模型训练工具链
│   ├── train.py                       # 一键训练
│   ├── number_cnn.onnx                # 导出模型
│   ├── batch_inference.py             # 批量推理测试
│   ├── test_augmentation.py           # 鲁棒性测试
│   └── visualize.py                   # 可视化结果
├── Camera/                            # 相机配置与日志
├── build/
└── install/
```

---

## 数字识别模型训练

```bash
cd DeepLearning
python train.py    # 数据增强 → 训练 → 保存 best.pth → 导出 ONNX
```

详见 `DeepLearning/训练手册.md`。
