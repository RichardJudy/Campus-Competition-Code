# Campus Competition Code

校园竞赛视觉检测代码 - USB相机装甲板检测与姿态解算

## 项目说明

这是一个精简后的视觉测试工程，用于校园竞赛中的装甲板检测任务。

运行时会输出每个装甲板的：

- 世界坐标 \((x, y, z)\)（单位：m）
- 距离（相对世界原点的欧氏距离，单位：m）
- 相对云台的 yaw / pitch（单位：deg，按像素差解算）


## 配置说明

主要配置文件：`configs/demo.yaml`，其中包括：

- `camera_matrix` / `distort_coeffs`：相机内参与畸变
- `image_width` / `image_height`：采集分辨率
- `R_camera2gimbal` / `t_camera2gimbal`：相机到云台标定
- `R_gimbal2imubody`：云台到 IMU 标定
- `threshold` 等传统检测参数
- `image_width` / `image_height` / `usb_frame_rate`：USB 相机参数

你只需要确保：

- USB 相机的内参标定结果已正确写入 `camera_matrix` / `distort_coeffs`
- 图像尺寸与真实采集分辨率一致

---

## 编译步骤

在项目根目录执行：

```bash
mkdir -p build
cd build
cmake ..
make usbcamera_detect_test -j$(nproc)
```

编译成功后，会生成：

- `build/usbcamera_detect_test`

---

## 运行方式

### 1. 确认相机设备号

先查看当前系统的 `/dev/videoX`：

```bash
v4l2-ctl --list-devices
```

示例输出：

```text
Integrated Camera: ...:
    /dev/video0
    /dev/video1

icspring camera: ...:
    /dev/video2
    /dev/video3
```

假设你要用的是 `/dev/video2`，则设备名为 `video2`。

### 2. 在工程根目录运行

```bash
cd /home/zyy/Desktop/sp_vision_25-main   # 换成你的工程路径

# 显示窗口 + 指定 USB 相机设备
./build/usbcamera_detect_test configs/demo.yaml -n=video2 -d
```

参数说明：

- 第一个无 `-` 前缀的参数：配置文件路径（这里是 `configs/demo.yaml`）
- `-n=video2` 或 `--name=video2`：指定 USB 相机设备，对应 `/dev/video2`
  - 注意要写成 `-n=video2`，不要写成 `-n video2`，否则 `CommandLineParser` 会解析错误
- `-d` 或 `--display`：开启图像显示窗口

也可以在 `build` 目录运行：

```bash
cd /home/zyy/Desktop/sp_vision_25-main/build
./usbcamera_detect_test ../configs/demo.yaml -n=video2 -d
```

---

## 输出含义

程序每检测到一个装甲板，会在日志中输出类似信息：

```text
装甲板 #123: 世界坐标(1.234, 0.567, 2.890), 距离=3.456m, yaw=5.23deg, pitch=-2.15deg
```

- `世界坐标(x, y, z)`：装甲板中心在世界坐标系下的位置（单位：m）
- `距离`：\(\sqrt{x^2 + y^2 + z^2}\)，相对世界原点（通常是云台旋转中心）直线距离
- `yaw` / `pitch`：
  - 使用装甲板图像中心与相机主点的像素差，通过针孔模型解算视线偏转角
  - yaw：水平偏转角（右偏为正）
  - pitch：俯仰角（抬头为负，低头为正，取决于坐标系约定）

如果加上 `-d`，还会弹出一个窗口：

- 以绿色点标出装甲板四个角点
- 在装甲板中心附近标注 `dist: Xm`

按 `q` 键可以退出程序。

---

## 常见问题

- **配置文件找不到**  
  确保运行时配置路径正确：
  - 在根目录：`configs/demo.yaml`
  - 在 `build` 目录：`../configs/demo.yaml`

- **USB 相机无法打开**  
  - 用 `v4l2-ctl --list-devices` 确认实际设备号
  - 用 `-n=videoX` 指定正确设备名
  - 检查是否有其他程序占用该相机

---

## GitHub 仓库

项目地址：https://github.com/richard_udy/Campus-Competition-Code

---

## 许可证

详见 [LICENSE](LICENSE) 文件

