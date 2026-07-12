# RK3576 C++ 快速目标跟踪 — 技术文档

## 1. 架构概览

```
USB相机  ──→  主循环(帧率不限)  ──→  串口/stdout $AIM输出
                  │                        ↑
                  ├─ 提交帧 → DetectorWorker(独立线程) ──→ RKNN异步推理
                  │                ↓
                  ├─ 取最新结果 ←── InferResult(非阻塞)
                  │
                  ├─ RectRefiner(ROI矩形精修,CPU)
                  └─ RectTracker(α-β滤波预测)
```

**核心设计思路**：不是每帧都跑YOLO，而是**粗粒度RKNN检测** + **轻量ROI精修** + **短时预测** 三管齐下，在RK3576上实现高帧率目标跟踪。

### 模块清单

| 模块 | 文件 | 职责 |
|------|------|------|
| `main.cpp` | `src/main.cpp` | 主循环：帧读取→检测调度→精修→跟踪→输出 |
| `config.hpp/cpp` | `include/rkfast/config.hpp`, `src/config.cpp` | 命令行解析，所有可调参数 |
| `types.hpp` | `include/rkfast/types.hpp` | 基础数据结构：Box, Detection, TrackSnapshot, InferTimings |
| `rknn_detector.hpp/cpp` | 两者 | RKNN模型加载、推理、NMS后处理、检测选择 |
| `camera_source.hpp/cpp` | 两者 | USB相机采集（独立线程，V4L2优先） |
| `tracker.hpp/cpp` | `include/rkfast/tracker.hpp`, `src/tracker.cpp` | α-β滤波跟踪器，支持预测/丢失/重搜索状态机 |
| `rect_refiner.hpp/cpp` | `include/rkfast/rect_refiner.hpp`, `src/rect_refiner.cpp` | 基于ROI的二值化+边缘轮廓提取，四角点排序 |
| `aim_writer.hpp/cpp` | `include/rkfast/aim_writer.hpp`, `src/aim_writer.cpp` | $AIM协议输出（串口/标准输出） |
| `rga_preprocess.hpp/cpp` | 两者 | RGA硬件加速预处理（resize+BGR2RGB+letterbox），支持OpenCV降级 |
| `thread_affinity.hpp/cpp` | 两者 | CPU核心绑定（pthread_setaffinity_np） |

---

## 2. 编译构建

### 2.1 板端依赖

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev
# RGA硬件加速（可选但强烈推荐）
sudo apt install -y librga-dev librga2
```

### 2.2 编译命令

**一键编译（推荐，带RGA）：**

```bash
cd ~/rk3576_rknn_targeting/cpp_fast_targeting
bash build_rga.sh
```

**手动编译：**

```bash
cd ~/rk3576_rknn_targeting/cpp_fast_targeting
mkdir -p build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DRKFAST_USE_RGA=ON \
  -DCMAKE_CXX_FLAGS="-march=native -mtune=cortex-a76"
make -j$(nproc)
```

**不带RGA编译（降级方案）：**

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release -DRKFAST_USE_RGA=OFF
make -j$(nproc)
```

CMakeLists.txt 会自动查找：
- OpenCV（优先 `find_package(OpenCV)`，失败则 `pkg-config opencv4`）
- RKNN API（`rknn_api.h` + `librknnrt.so`，搜索路径包含 `third_party/rknn/` 和系统路径）
- RGA（`librga.so`，可选，通过 `RKFAST_USE_RGA` 控制）
- 编译期检测 `rknn_set_core_mask` API 是否存在（`RKFAST_RKNN_HAS_CORE_MASK`）

Release 编译选项：`-O3 -ffast-math -fno-math-errno -fno-trapping-math`

### 2.3 项目目录结构

```
cpp_fast_targeting/
├── CMakeLists.txt
├── build_rga.sh              # 一键编译脚本
├── include/rkfast/
│   ├── aim_writer.hpp
│   ├── camera_source.hpp
│   ├── config.hpp
│   ├── rect_refiner.hpp
│   ├── rga_preprocess.hpp
│   ├── rknn_detector.hpp
│   ├── thread_affinity.hpp
│   ├── tracker.hpp
│   └── types.hpp
├── src/
│   ├── aim_writer.cpp
│   ├── camera_source.cpp
│   ├── config.cpp
│   ├── main.cpp
│   ├── rect_refiner.cpp
│   ├── rga_preprocess.cpp
│   ├── rknn_detector.cpp
│   ├── thread_affinity.cpp
│   └── tracker.cpp
├── third_party/rknn/include/
│   └── rknn_api.h            # 备用RKNN头文件
└── build/
    └── rkfast_target          # 编译产物
```

---

## 3. 启动命令

### 3.1 前置步骤：性能模式

**每次开机后执行一次：**

```bash
cd ~/rk3576_rknn_targeting
sudo bash tools/rk3576_performance.sh
```

此脚本将 CPU governor 设为 `performance`、GPU/NPU devfreq governor 设为 `performance`，并设置 RGA 设备权限。

### 3.2 一键启动

```bash
cd ~/rk3576_rknn_targeting
bash quick_start.sh
```

该脚本自动完成：性能模式 → 编译检查 → 运行程序。

### 3.3 完整命令行参数

```bash
cd ~/rk3576_rknn_targeting/cpp_fast_targeting/build
./rkfast_target \
  --model ../../models/best_yolo26n_512_rk3576_int8.rknn \
  --camera /dev/video73 \
  --width 640 --height 480 --fps 60 \
  --fourcc MJPG \
  --input-size 512 \
  --core-mask 01 \
  --class-count 1 \
  --target-class 0 \
  --conf 0.45 \
  --iou 0.50 \
  --max-det 1 \
  --detect-interval-locked 2 \
  --max-predict-misses 3 \
  --track-alpha 0.55 --track-beta 0.20 --track-box-alpha 0.45 \
  --main-affinity 4-7 --detector-affinity 4-7 --camera-affinity 0-3 \
  --serial-port /dev/ttyUSB0 --baudrate 115200 \
  --no-window --stats-interval 2.0
```

### 3.4 关键参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--model` | `../../models/best_rk3576_int8.rknn` | RKNN模型路径 |
| `--camera` | `/dev/video73` | 相机设备（支持数字索引或路径） |
| `--width/--height` | 640/480 | 相机采集分辨率 |
| `--fps` | 60 | 相机帧率设置 |
| `--fourcc` | MJPG | 相机编码格式（MJPG通常比YUYV快） |
| `--input-size` | 640 | 模型输入尺寸（512可提升速度） |
| `--core-mask` | default | NPU核心：`0`/`1`/`01`/`012`/`auto`/`all` |
| `--detect-interval-locked` | 2 | 锁定后跳帧数（每N帧才触发RKNN检测） |
| `--max-predict-misses` | 3 | 连续丢失几次后回到SEARCHING |
| `--main-affinity` | (空) | 主线程CPU绑定，如 `4-7` 绑定大核 |
| `--detector-affinity` | (空) | 推理线程CPU绑定 |
| `--camera-affinity` | (空) | 相机线程CPU绑定 |
| `--serial-port` | (空) | 串口设备路径，为空则输出到stdout |
| `--window/--no-window` | no-window | 是否显示OpenCV调试窗口 |
| `--stats-interval` | 1.0 | 性能统计打印间隔（秒） |

### 3.4 基准测试命令

```bash
# 单核 vs 双核 vs 自动对比
for mask in 0 1 01 auto; do
  echo "===== core-mask ${mask} ====="
  timeout 20s ./rkfast_target \
    --model ../../models/best_yolo26n_512_rk3576_int8.rknn \
    --camera /dev/video73 --width 640 --height 480 --fps 60 \
    --fourcc MJPG --input-size 512 --class-count 1 --target-class 0 \
    --max-det 1 --no-window --core-mask "${mask}"
done
```

---

## 4. 输出格式

### 4.1 性能统计（stdout）

```
stats frame_fps=50.2 infer_fps=58.1 pre=1.5ms rknn=7.4ms post=0.2ms select=0.0ms total=9.3ms state=LOCKED conf=0.85
```

| 字段 | 含义 |
|------|------|
| `frame_fps` | 主循环帧率（受限于相机+处理延迟） |
| `infer_fps` | RKNN推理吞吐（异步，通常高于frame_fps） |
| `pre` | 预处理耗时（RGA: 1-2ms, OpenCV: 8-10ms） |
| `rknn` | NPU推理耗时 |
| `post` | 后处理耗时（解码+坐标映射） |
| `select` | 检测选择耗时（prior box匹配） |
| `total` | 单次推理总耗时 |
| `state` | SEARCHING / LOCKED / LOST |
| `conf` | 当前跟踪置信度 |

### 4.2 $AIM 协议（串口/stdout）

```
$AIM,1,LOCKED,320.0,240.0,-0.0,0.0,0.852,roi+rknn
```

字段：`$AIM,found,state,cx,cy,dx,dy,conf,source`

- `found`: 1=锁定, 0=搜索中
- `cx,cy`: 目标中心绝对坐标
- `dx,dy`: 相对画面中心的偏移
- `source`: `rknn` / `roi` / `roi+rknn` / `predict` / `none`

---

## 5. 踩坑记录

### 5.1 编译期坑

#### 坑1: `rga_preprocessor_` 未声明

**现象**：`error: 'rga_preprocessor_' was not declared in this scope`

**原因**：`rknn_detector.hpp` 中缺少 `RgaPreprocessor` 成员变量声明和头文件包含。

**解决**：在 `include/rkfast/rknn_detector.hpp` 中添加：
```cpp
#include "rkfast/rga_preprocess.hpp"
// 在 RknnDetector 类的 private 区域添加：
RgaPreprocessor rga_preprocessor_;
```

#### 坑2: RGA API 参数不匹配

**现象**：`error: too few arguments to function 'imresize_t'`

**原因**：不同RGA库版本的 `imresize_t` 签名不同（v1无sync参数，v2有sync参数），且 `wrapbuffer_virtualaddr` 可能需要额外stride参数。

**解决**：在 `src/rga_preprocess.cpp` 中使用带 sync 参数的调用：
```cpp
imresize_t(src, dst, scale_val, scale_val, INTER_LINEAR, 1);  // sync=1
```

如果仍有问题，检查头文件中的实际签名：
```bash
grep -A 3 "imresize_t" /usr/include/rga/im2d_single.h
```

#### 坑3: CMake 找不到 OpenCV

**现象**：`find_package(OpenCV)` 失败，`pkg-config opencv4` 也失败。

**解决**：
```bash
sudo apt install libopencv-dev
pkg-config --modversion opencv4   # 验证
```

#### 坑4: CMake 找不到 rknn_api.h

**现象**：`FATAL_ERROR: Could not find rknn_api.h`

**解决**：将 `rknn_api.h` 放到 `cpp_fast_targeting/third_party/rknn/include/` 目录下，CMakeLists.txt 会优先搜索该路径。

### 5.2 运行时坑

#### 坑5: RGA 初始化失败（pre=9ms 慢速）

**现象**：
```
RGA init failed (RGA init failed: -1), using OpenCV fallback
stats ... pre=9.0ms ...
```

**原因**：`/dev/rga` 设备节点不存在或权限不足。

**诊断**：
```bash
bash test_rga.sh                          # 运行诊断脚本
ls -l /dev/rga                           # 检查设备节点
ls /sys/devices/platform/*rga/dev        # 查找设备号
```

**解决**：
```bash
# 方案1：手动创建设备节点
cat /sys/devices/platform/27920000.rga/dev   # 输出如 241:0
sudo mknod /dev/rga c 241 0
sudo chmod 666 /dev/rga

# 方案2：添加udev规则（永久生效）
sudo tee /etc/udev/rules.d/99-rga.rules << 'EOF'
KERNEL=="rga", MODE="0666", GROUP="video"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG video $USER
newgrp video
```

#### 坑6: 模型文件找不到

**现象**：`could not open model: ../../models/best_rk3576_int8.rknn`

**原因**：模型文件不存在或相对路径计算错误（从 `cpp_fast_targeting/build/` 运行时，`../../models/` 指向项目根目录的 `models/`）。

**解决**：
```bash
# 检查模型是否存在
find ~/rk3576_rknn_targeting -name '*.rknn'
# 使用绝对路径或确保相对路径正确
./rkfast_target --model /home/lckfb/rk3576_rknn_targeting/models/xxx.rknn ...
```

#### 坑7: 相机打开失败

**现象**：`could not open camera: /dev/video73`

**解决**：
```bash
ls -l /dev/video*                    # 列出可用相机
v4l2-ctl --list-devices              # 查看相机名称
v4l2-ctl -d /dev/video0 --list-formats-ext  # 查看支持的格式
# 使用正确的设备号和FOURCC
./rkfast_target --camera /dev/video0 --fourcc MJPG ...
```

#### 坑8: 帧率上不去（被sleep限制）

**现象**：帧率被限制在约30fps，远低于理论值。

**原因**：旧版本在主循环和相机线程中使用了 `std::this_thread::sleep_for()` 。

**解决**（已在当前代码中修复）：
- `main.cpp:128`：重复帧检测时改为直接 `continue`，不睡眠
- `camera_source.cpp:102`：空帧时改用 `std::this_thread::yield()` 代替 `sleep_for(3ms)`
- 设置 `cap_.set(cv::CAP_PROP_BUFFERSIZE, 1)` 使用最小缓冲

#### 坑9: USB相机带宽瓶颈

**现象**：帧率受限于USB带宽，640x480@MJPG可到60fps，但更高分辨率/YUYV格式大幅降低。

**建议**：
- 优先使用 MJPG 编码（`--fourcc MJPG`）
- 采集分辨率和模型输入尺寸可以不同（640x480采集 + 512模型输入）
- 采集低分辨率可换取更高帧率（`--width 320 --height 240`）

#### 坑10: RK3576 大小核调度

**现象**：推理线程跑在小核上导致 `rknn_ms` 偏高。

**解决**：使用 CPU affinity 将推理和主循环绑定到大核：
```bash
# 查看CPU拓扑（RK3576通常0-3小核，4-7大核）
lscpu -e
# 绑定
./rkfast_target --main-affinity 4-7 --detector-affinity 4-7 --camera-affinity 0-3 ...
```

### 5.3 设计与调试建议

#### 坑11: detect_interval_locked 理解

`detect_interval_locked=2` 表示"锁定后每2帧才做一次RKNN推理"，中间帧用ROI精修或预测。这个值设太大会导致跟踪漂移，设太小浪费NPU算力。建议值：2-5。

#### 坑12: track_alpha / track_beta 调参

追踪器使用 α-β 滤波器：
- `track_alpha=0.55`：位置平滑系数，越大响应越快但越抖
- `track_beta=0.20`：速度估计系数，越大对速度变化越敏感
- `track_box_alpha=0.45`：边界框尺寸平滑

目标运动快时增大 alpha/beta，静止目标可减小。

#### 坑13: 查看实时性能

```bash
# 监控NPU频率
watch -n 1 cat /sys/class/devfreq/fdab0000.npu/cur_freq
# 监控CPU频率
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq
# 用perf分析热点
sudo perf record -g ./rkfast_target ... --no-window
sudo perf report
```

---

## 6. 性能数据参考

| 配置 | pre | rknn | total | frame_fps |
|------|-----|------|-------|-----------|
| OpenCV + 双核NPU + 640模型 | 9.0ms | 7.6ms | 17.1ms | 27-30 |
| **RGA + 双核NPU + 512模型** | **1.5ms** | **7.4ms** | **9.3ms** | **48-55** |
| RGA + 三核NPU + 512模型 | 1.5ms | 5.0ms | 6.8ms | 65-75 |
| RGA + 三核 + 416模型(理论) | 1.2ms | 3.5ms | 5.0ms | 80-100 |

**结论**：RGA是最大瓶颈突破点（9ms→1.5ms），其次是模型输入尺寸缩小，三是NPU多核调度。

---

## 7. 快速故障排查流程图

```
启动失败?
├─ 找不到rknn_api.h → 放到 third_party/rknn/include/
├─ 找不到librknnrt → 检查 /usr/lib/librknnrt.so
├─ 找不到OpenCV → sudo apt install libopencv-dev
├─ RGA编译不过 → cmake -DRKFAST_USE_RGA=OFF 先跑起来
└─ 模型找不到 → 用绝对路径 --model /home/xxx/xxx.rknn

运行正常但帧率低?
├─ pre=9ms → RGA未生效，检查 /dev/rga 权限
├─ rknn > 15ms → 尝试 --core-mask 01 或 012
├─ frame_fps低但infer_fps高 → 相机瓶颈，换MJPG或降分辨率
└─ 整体慢 → 确认已运行 performance.sh

输出问题?
├─ 串口无输出 → 检查 --serial-port 路径和波特率
├─ stdout无$AIM → 无串口时自动输出到stdout（间隔0.2s）
└─ state一直SEARCHING → 检查模型类别 --target-class --class-count
```
