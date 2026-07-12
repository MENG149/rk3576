# RK3576 RKNN 二维云台目标检测

面向 RK3576 的 USB 相机实时目标检测与二维云台输出工程。当前推荐运行路径是
`cpp_fast_targeting`：C++17、RKNN Runtime、RGA 预处理、FP16 模型和双 NPU
核心。

## 实测结果

测试环境：RK3576、Linux 6.1.99、librknnrt 2.3.2、512x512 FP16 模型。

| 输入路径 | 推理 FPS | set | rknn | total |
|---|---:|---:|---:|---:|
| legacy rknn_inputs_set | 43.6 | 10.0 ms | 10.1 ms | 22.8 ms |
| FP16 zero-copy | 73.3 | 1.1 ms | 9.8 ms | 13.6 ms |

207 张验证图片的新旧路径检测结果完全一致。相机本身当前限制在约 27.5 FPS，
所以 frame_fps 不会随推理上限同步提高。

## 默认流水线

```text
USB camera
  -> RGA/OpenCV letterbox RGB
  -> UINT8 / 255 lookup conversion to native FP16
  -> RKNN zero-copy input
  -> FP16 output decode
  -> rectangle refine and tracker
  -> $AIM stdout/serial output
```

默认设置：

- 模型：`models/best_n_stage1_512_fp16.rknn`
- 相机：`/dev/video73`
- NPU：core 0 + core 1，mask `01`
- CPU/NPU/DDR：performance governor，不使用 CPU userspace 固定锁频
- RKNN 输入：`auto`，零拷贝不可用时自动回退 legacy

## 目录

- `cpp_fast_targeting/`：推荐的 C++ 板端运行时
- `models/best_n_stage1_512_fp16.rknn`：已验证可用模型
- `tools/rk3576_performance.sh`：板端性能 governor 设置
- `docs/`：设计、部署和学习文档
- `rknn_target/`、`run_usb_rknn.py`：早期 Python 运行时，保留作参考

## 板端依赖

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev
```

板端还需要与系统和 NPU 驱动匹配的 `librknnrt.so`、`rknn_api.h`，以及
RGA 运行库。项目包含 RKNN 2.3.2 头文件回退：
`cpp_fast_targeting/third_party/rknn/include/rknn_api.h`。

## 编译

```bash
cd ~/rk3576_rknn_targeting/cpp_fast_targeting
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DRKFAST_USE_RGA=ON \
  -DRKNN_API_INCLUDE_DIR=/usr/include \
  -DRKNN_API_LIB=/usr/lib/librknnrt.so
cmake --build build --parallel "$(nproc)"
```

如果系统没有安装 RKNN 头文件，可省略 `RKNN_API_INCLUDE_DIR`，CMake 会使用
项目内的头文件。

## 运行

```bash
cd ~/rk3576_rknn_targeting/cpp_fast_targeting
./run_camera.sh
```

默认启动会通过 sudo 应用 performance governor，然后启用 FP16 zero-copy 和
NPU 双核。常用参数：

```bash
# 跳过系统 governor 设置
./run_camera.sh --no-perf

# 强制旧输入路径，便于回归对比
./run_camera.sh --input-mode legacy

# 串口输出
./run_camera.sh --serial /dev/ttyUSB0
```

## 验证

CPU 单元测试：

```bash
cd ~/rk3576_rknn_targeting/cpp_fast_targeting
cmake -S tests -B tests/build -DCMAKE_BUILD_TYPE=Release
cmake --build tests/build --parallel "$(nproc)"
ctest --test-dir tests/build --output-on-failure
```

图片级 legacy/zero-copy 对比：

```bash
./build/input_mode_compare \
  --model ../models/best_n_stage1_512_fp16.rknn \
  --images /path/to/validation/images \
  --input-size 512 --class-count 1 --target-class 0 \
  --conf 0.25 --core-mask 01
```

对比器会检查输入校验值、检出状态、类别、置信度和框坐标，任何差异都会返回
非零状态。

## 详细文档

- `docs/superpowers/specs/2026-07-12-rknn-input-zero-copy-design.md`
- `RK3576_矩形识别修复与性能优化报告_2026-07-12.md`
- `RK3576_CPP_技术文档.md`
- `RK3576_板端启动指南.md`
