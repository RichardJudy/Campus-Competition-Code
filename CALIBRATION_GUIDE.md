# 相机标定使用说明

## 1. 准备标定板

准备一个棋盘格标定板，确保：
- 标定板平整，无弯曲
- 标定板格点数量与配置文件中的 `pattern_cols` 和 `pattern_rows` 匹配
- 标定板格点间距与配置文件中的 `center_distance_mm` 匹配（单位：毫米）

## 2. 配置标定板参数

编辑 `configs/calibration.yaml` 文件：

```yaml
pattern_cols: 11          # 标定板列数
pattern_rows: 8           # 标定板行数
center_distance_mm: 30    # 标定板格点间距（毫米）
pattern_type: "chessboard" # 标定板类型："chessboard" 或 "circles"
```

## 3. 拍摄标定图片

### 编译程序

```bash
mkdir build
cd build
cmake ..
make capture
```

### 运行拍摄程序

```bash
# 使用默认参数运行
./capture assets/img_with_q

# 或指定相机设备
./capture assets/img_with_q -n video0

# 或指定配置文件路径
./capture assets/img_with_q -c configs/calibration.yaml
```

### 拍摄操作

程序启动后会打开相机窗口，显示实时画面：

- **按 's' 键**：保存当前帧为标定图片
- **按 'c' 键**：清空输出文件夹中的所有图片
- **按 'q' 键或 ESC 键**：退出程序

### 拍摄建议

为了获得准确的标定结果，建议：
- 拍摄 **15-30 张** 不同角度和位置的标定板图片
- 标定板应覆盖相机视场的不同区域（左上、右上、左下、右下、中心等）
- 标定板距离相机的距离应覆盖实际使用范围
- 确保每张图片都能清晰识别标定板

## 4. 运行标定程序

### 编译标定程序

```bash
make calibrate_camera
```

### 运行标定

```bash
# 使用默认参数运行
./calibrate_camera assets/img_with_q

# 或指定配置文件路径
./calibrate_camera assets/img_with_q -c configs/calibration.yaml
```

### 标定过程

程序会：
1. 逐张读取标定图片
2. 显示识别结果窗口
3. 按任意键继续下一张图片
4. 完成所有图片后，输出标定结果

### 标定结果

程序会输出类似以下内容：

```yaml
# 重投影误差: 0.2584px
camera_matrix: [2232.5056665823868, 0, 781.35389439373671, 0, 2235.0406464158923, 589.72206701993866, 0, 0, 1]
distort_coeffs: [-0.035302848036215723, 0.047564957843643611, 0.0010940289390717367, 0.0066584819278176915, 0]
```

## 5. 应用标定结果

将标定结果复制到 `configs/demo.yaml` 文件中进行替换：

```yaml
# camera calibrations
camera_matrix: [2232.5056665823868, 0, 781.35389439373671, 0, 2235.0406464158923, 589.72206701993866, 0, 0, 1]
distort_coeffs: [-0.035302848036215723, 0.047564957843643611, 0.0010940289390717367, 0.0066584819278176915, 0]
```

