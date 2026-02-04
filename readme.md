# Campus Competition Code

校赛用的装甲板检测系统，基于 USB 相机和传统视觉算法，能够实时检测装甲板并输出世界坐标、距离和 yaw/pitch 角度。

## 项目结构

```
├── configs/          # 配置文件目录
│   └── demo.yaml     # 主配置文件
├── io/               # 输入输出模块
│   └── usbcamera/    # USB 相机驱动
├── tasks/            # 任务模块
│   └── auto_aim/     # 自动瞄准模块
│       ├── armor.cpp     # 装甲板类
│       ├── detector.cpp  # 检测器
│       └── solver.cpp    # 解算器
├── tests/            # 测试代码
│   └── usbcamera_detect_test.cpp  # USB 相机检测测试
├── tools/            # 工具类
│   ├── img_tools.cpp     # 图像处理工具
│   ├── logger.cpp        # 日志工具
│   └── math_tools.cpp    # 数学工具
├── CMakeLists.txt    # CMake 配置文件
└── readme.md         # 项目说明
```

## 依赖项

项目依赖以下库：
- OpenCV
- fmt
- Eigen3
- spdlog
- yaml-cpp
- nlohmann_json

## 配置

配置文件位于 `configs/demo.yaml`，主要配置项包括：

### 基本配置
- `enemy_color`: 敌方颜色，可选值："red" 或 "blue"
- `use_traditional`: 是否使用传统视觉算法，设为 true

### 相机配置
- `image_width`/`image_height`: 图像分辨率
- `usb_exposure`: USB 相机曝光值
- `usb_frame_rate`: USB 相机帧率
- `usb_gamma`: USB 相机伽马值
- `usb_gain`: USB 相机增益

### 相机标定参数
- `camera_matrix`: 相机内参矩阵
- `distort_coeffs`: 相机畸变系数
- `R_camera2gimbal`: 相机到云台的旋转矩阵
- `t_camera2gimbal`: 相机到云台的平移向量

### 检测器配置
- `threshold`: 阈值
- `max_angle_error`: 最大角度误差
- `min_lightbar_ratio`/`max_lightbar_ratio`: 灯条比例范围
- `min_lightbar_length`: 最小灯条长度
- `min_armor_ratio`/`max_armor_ratio`: 装甲板比例范围
- `max_side_ratio`: 最大边长比例
- `max_rectangular_error`: 最大矩形误差
- `min_confidence`: 最小置信度

### 其他配置
- `roi`: 感兴趣区域
- `tracker`: 追踪器参数
- `aimer`: 瞄准器参数
- `shooter`: 发射器参数

## 编译

```bash
# 创建并进入构建目录
mkdir -p build && cd build

# 配置 CMake
cmake ..

# 编译测试程序
make usbcamera_detect_test -j$(nproc)
```

可执行文件将生成在 `build/usbcamera_detect_test`。

## 运行

### 1. 查看相机设备

```bash
v4l2-ctl --list-devices
```

### 2. 运行测试程序

#### 在项目根目录运行

```bash

./build/usbcamera_detect_test configs/demo.yaml -n=video2 -d

```


#### 在 build 目录运行

```bash

../configs/demo.yaml -n=video2 -d

```

### 命令行参数说明

- `configs/demo.yaml`: 配置文件路径
- `-n=video2`: USB 相机设备名称
- `-d`: 显示视频流和检测结果，按 q 退出
- `-h`: 显示帮助信息

## 输出

### 日志输出

检测到装甲板时，会在控制台输出一行日志，格式如下：
```
装甲板 #123: 世界坐标(1.234, 0.567, 2.890), 距离=3.456m, yaw=5.23deg, pitch=-2.15deg
```
其中：
- `#123`: 帧序号
- `世界坐标(1.234, 0.567, 2.890)`: 目标在世界坐标系中的坐标 (x, y, z)
- `距离=3.456m`: 目标距离相机的直线距离
- `yaw=5.23deg`: 偏航角（左右方向）
- `pitch=-2.15deg`: 俯仰角（上下方向）

### 可视化输出

使用 `-d` 参数运行时，会显示一个名为 "Detection Result" 的窗口，窗口中：
- 绿色点标记装甲板的四个角点
- 装甲板中心显示距离信息

## 常见问题和解决方案

### 1. 相机打不开

- 确认相机设备号是否正确
- 检查相机是否被其他程序占用
- 检查 USB 连接是否稳定

### 2. 检测效果不佳

- 调整 `configs/demo.yaml` 中的 `threshold` 值
- 调整相机曝光、增益等参数
- 确保环境光线充足且稳定

### 3. 运行时崩溃

- 检查配置文件路径是否正确
- 检查相机设备是否存在
- 检查相机参数是否设置合理

## 贡献指南

欢迎提交 Issue 和 Pull Request 来帮助改进这个项目！

---

*祝你在比赛中取得好成绩！*
