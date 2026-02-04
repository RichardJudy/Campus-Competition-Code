# Campus Competition Code

校赛用的装甲板检测，USB 相机 + 传统视觉，输出世界坐标、距离和 yaw/pitch。

## 配置

改 `configs/demo.yaml` 就行。相机内参、畸变、分辨率、云台标定都在里面，标定完把 `camera_matrix`、`distort_coeffs` 填对，分辨率跟实际采集一致。

## 编译

```bash
mkdir -p build && cd build
cmake ..
make usbcamera_detect_test -j$(nproc)
```

可执行文件在 `build/usbcamera_detect_test`。

## 运行

先看下相机是哪个设备：

```bash
v4l2-ctl --list-devices
```

比如是 `/dev/video2` 就写 `video2`。在项目根目录：

```bash
./build/usbcamera_detect_test configs/demo.yaml -n=video2 -d
```

注意 `-n=video2` 必须带等号，写成 `-n video2` 会解析错。`-d` 是开显示窗口，按 q 退出。

在 build 里跑的话用 `../configs/demo.yaml`。

## 输出

检测到装甲板会打一行日志，例如：

```
装甲板 #123: 世界坐标(1.234, 0.567, 2.890), 距离=3.456m, yaw=5.23deg, pitch=-2.15deg
```

世界坐标和距离是 PnP 解算再转到世界系的，yaw/pitch 是像素差按针孔模型算的视线角。开 `-d` 会在图上画角点和距离。

## 踩坑

- 配置文件路径：根目录用 `configs/demo.yaml`，在 build 里用 `../configs/demo.yaml`。
- 相机打不开：用 v4l2 看准设备号，`-n=videoX` 写对，看有没有别的程序占着。

---

Repo: https://github.com/richard_udy/Campus-Competition-Code  
License 见 [LICENSE](LICENSE)。
