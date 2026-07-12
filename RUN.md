# RK3576 RKNN 运行说明

这份文档只说明如何在 RK3576 板子上运行当前精简版程序。

## 1. 进入项目目录

把项目放到 RK3576 板子后，先进入目录：

```bash
cd /path/to/rk3576_rknn_targeting
```

如果你直接把整个目录拷到板子的用户目录，例如 `/home/firefly/rk3576_rknn_targeting`：

```bash
cd /home/firefly/rk3576_rknn_targeting
```

## 2. 检查 RKNN 模型

默认模型路径是：

```text
models/best_rk3576_int8.rknn
```

检查模型是否存在：

```bash
ls -lh models/best_rk3576_int8.rknn
```

如果模型不在这个路径，可以运行时手动指定：

```bash
--model /your/path/best_rk3576_int8.rknn
```

## 3. 安装依赖

在 RK3576 板子上执行：

```bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy python3-serial
pip3 install -r requirements-rk3576.txt
```

如果 `rknn-toolkit-lite2` 安装失败，需要使用和板子系统、Python 版本匹配的 Rockchip aarch64 wheel 包安装。

## 4. 确认摄像头和串口

查看 USB 摄像头设备：

```bash
ls /dev/video*
```

查看串口设备：

```bash
ls /dev/ttyUSB* /dev/ttyAMA* 2>/dev/null
```

当前脚本默认示例使用：

```text
camera: /dev/video73
serial: /dev/ttyUSB0
```

如果你的设备号不同，把后面命令里的 `/dev/video73` 和 `/dev/ttyUSB0` 改成实际设备。

## 5. 语法检查

先确认 Python 文件没有语法问题：

```bash
python3 -m py_compile run_usb_rknn.py rknn_target/*.py
```

## 6. 设置板子性能模式

正式跑之前建议先拉高 CPU/NPU/DDR 频率：

```bash
sudo bash tools/rk3576_performance.sh
```

## 7. 正式高 FPS 运行

推荐直接执行：

```bash
bash run_fast_rk3576.sh
```

等价完整命令：

```bash
python3 run_usb_rknn.py \
  --model models/best_rk3576_int8.rknn \
  --camera /dev/video73 --width 640 --height 480 --fps 60 \
  --fourcc MJPG --fast --workers 2 --worker-core-masks 0,1 \
  --no-window --output-on-result \
  --serial-port /dev/ttyUSB0 --baudrate 115200 --serial-interval 0.02
```

这个模式不开窗口，优先保证 FPS。

## 8. 打开本地窗口看摄像头

如果板子接了显示器，或者你能显示 OpenCV 窗口，用下面命令：

```bash
python3 run_usb_rknn.py \
  --model models/best_rk3576_int8.rknn \
  --camera /dev/video73 --width 640 --height 480 --fps 60 \
  --fourcc MJPG --fast --workers 2 --worker-core-masks 0,1 \
  --window --display-interval 2 --output-on-result \
  --serial-port /dev/ttyUSB0 --baudrate 115200 --serial-interval 0.02
```

窗口里会显示摄像头画面、画面中心、目标框、目标中心和 FPS。

退出窗口：

```text
按 q 或 Esc
```

## 9. 没有串口时先看 stdout

如果只是先验证识别，不接串口，可以去掉串口参数：

```bash
python3 run_usb_rknn.py \
  --model models/best_rk3576_int8.rknn \
  --camera /dev/video73 --width 640 --height 480 --fps 60 \
  --fourcc MJPG --fast --workers 2 --worker-core-masks 0,1 \
  --no-window --output-on-result
```

程序会在终端输出 `$AIM` 文本。

## 10. 输出格式

程序只输出 `$AIM` 文本协议：

```text
$AIM,1,LOCKED,320.4,241.6,-12.0,8.0,0.876,rknn
```

字段含义：

```text
prefix,found,state,cx,cy,dx_px,dy_px,confidence,source
```

说明：

- `found`: `1` 表示有目标，`0` 表示没有目标。
- `state`: `LOCKED`、`LOST` 或 `SEARCHING`。
- `cx,cy`: 目标中心像素坐标。
- `dx_px,dy_px`: 目标中心相对画面中心的像素偏移。
- `confidence`: 模型置信度。
- `source`: `rknn` 或 `predict`。

## 11. 测试不同 NPU/worker 组合

比较不同调度方式：

```bash
bash tools/benchmark_rk3576_modes.sh
```

输出里重点看：

```text
stats frame_fps=... infer_fps=... pre=...ms rknn=...ms post=...ms select=...ms total=...ms
```

判断方法：

- `rknn` 时间最大：瓶颈在模型或 NPU runtime。
- `post` 时间大：后处理较慢，保持 `--fast --max-det 1`。
- `frame_fps` 低：摄像头实际帧率、USB 带宽或窗口显示可能拖慢。

## 12. 常见问题

### 找不到摄像头

报错类似：

```text
could not open USB camera
```

处理：

```bash
ls /dev/video*
```

然后把命令里的 `--camera /dev/video73` 改成实际设备。

### 找不到模型

报错类似：

```text
RKNN model not found
```

处理：

```bash
ls -lh models/
```

确认 `best_rk3576_int8.rknn` 是否存在，或者用 `--model` 指定实际路径。

### 缺少 RKNNLite

报错类似：

```text
Missing rknn-toolkit-lite2
```

处理：安装匹配 RK3576 板子系统和 Python 版本的 `rknn-toolkit-lite2` aarch64 wheel。

### 窗口打不开

如果是 SSH 无桌面环境，不能使用 `--window`。正式运行使用：

```bash
bash run_fast_rk3576.sh
```

### FPS 低

优先确认使用了：

```bash
sudo bash tools/rk3576_performance.sh
```

然后用无窗口模式：

```bash
bash run_fast_rk3576.sh
```

如果仍然低，再测试：

```bash
bash tools/benchmark_rk3576_modes.sh
```
