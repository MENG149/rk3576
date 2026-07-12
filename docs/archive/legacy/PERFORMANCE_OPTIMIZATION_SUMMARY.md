# RK3576 RKNN Targeting - 性能优化总结

## 📊 当前性能分析

**基准数据:**
```
frame_fps=27.5  (主循环帧率)
infer_fps=57.3  (推理帧率,多核已优化)
pre=9.0ms       (预处理 - 最大瓶颈)
rknn=7.6ms      (RKNN推理)
post=0.2ms      (后处理)
total=17.1ms    (总耗时)
```

**瓶颈识别:**
- ❌ **预处理占53%时间** (9.0ms / 17.1ms)
- ✅ 推理已优化 (多核infer_fps=57.3)
- ✅ 后处理很快 (0.2ms)

---

## 🎯 已实现的优化

### 1. ✅ RGA硬件加速 (最重要)

**实现内容:**
- 新增 `rga_preprocess.hpp/cpp` 模块
- 使用Rockchip RGA 2D硬件加速器
- 硬件执行: resize + letterbox + BGR→RGB转换
- 自动降级到OpenCV (如果RGA不可用)

**预期提升:**
```
预处理: 9.0ms → 1.5ms  (节省7.5ms)
帧率: 27.5 FPS → 48-52 FPS  (+80%)
```

**编译启用:**
```bash
cd cpp_fast_targeting
bash build_rga.sh
```

**运行时验证:**
```
RGA hardware acceleration enabled      # 编译时检测到RGA
RGA hardware acceleration initialized  # 运行时初始化成功
pre=1.5ms  # 预处理时间大幅降低
```

---

### 2. ✅ 编译器优化标志

**已启用 (CMakeLists.txt):**
```cmake
-O3                # 最高优化级别
-ffast-math        # 快速浮点运算
-fno-math-errno    # 跳过errno检查
-fno-trapping-math # 禁用浮点异常
```

**可选增强 (build_rga.sh):**
```cmake
-march=native      # 使用所有CPU特性
-mtune=cortex-a76  # 针对RK3576优化
```

---

### 3. ✅ 多核RKNN推理

**当前状态:**
- infer_fps=57.3 > frame_fps=27.5
- 说明RKNN已使用多核,不是瓶颈

**测试不同核心配置:**
```bash
# 单核
./rkfast_target --core-mask 0

# 双核 (可能当前配置)
./rkfast_target --core-mask 01

# 三核
./rkfast_target --core-mask 012

# 自动调度
./rkfast_target --core-mask auto
```

---

## 🚀 进一步优化建议

### 4. 降低模型分辨率 (如果精度允许)

**当前:** 640x640 输入

**优化方案:**
```python
# 重新导出RKNN模型
# 480x480: ~40% faster
# 416x416: ~50% faster  
# 320x320: ~70% faster

rknn.config(
    target_platform='rk3576',
    quantized_dtype='asymmetric_quantized-8',
    optimization_level=3,
    input_size_list=[[1, 3, 416, 416]]  # 降低分辨率
)
```

**预期提升:**
```
rknn: 7.6ms → 3-4ms  (节省4ms)
总帧率: 52 FPS → 65-70 FPS
```

---

### 5. 零拷贝相机捕获

**当前问题:**
- `camera_source.cpp:106` 使用 `std::move(frame)`
- OpenCV可能仍有内存分配

**优化方案:**
```cpp
// 方案A: 使用V4L2 MMAP
cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);  // 已完成
// 添加直接V4L2访问,避免OpenCV封装

// 方案B: 使用DMA buffer
// 相机 → DMA → RGA → RKNN (全程零拷贝)
```

**预期提升:** 0.5-1ms

---

### 6. 流水线并行化

**当前架构:**
```
相机线程 → 主线程 → RKNN工作线程
            ↓
          Tracker/Refiner (串行)
```

**优化架构:**
```cpp
// 将rect_refiner移到独立线程
RectRefinerWorker refiner_worker;

while (true) {
  frame = camera.ReadLatest();
  result = worker.LatestResult();     // 非阻塞
  refiner_worker.Submit(frame);       // 异步精化
  tracker.Update(...);
}
```

**预期提升:** 1-2ms (refiner与RKNN重叠)

---

### 7. 禁用不必要功能

```bash
# 已禁用窗口渲染
./rkfast_target --no-window

# 减少统计输出频率
./rkfast_target --stats-interval 5.0

# 如果不需要ROI精化,可以跳过refiner
# 编辑 main.cpp:148-154
```

**预期提升:** 0.2-0.5ms

---

### 8. 性能分析工具

```bash
# 在RK3576板上使用perf
sudo perf record -g ./rkfast_target --no-window
# 运行30秒后 Ctrl+C
sudo perf report

# 查找热点:
# - cv::resize / cv::cvtColor (应该消失,被RGA替代)
# - rknn_run
# - mutex锁竞争
```

---

## 📈 性能提升路线图

| 优化项 | 时间节省 | 累计FPS | 优先级 |
|--------|----------|---------|--------|
| 基准 | - | 27.5 | - |
| ✅ **RGA加速** | **-7.5ms** | **48-52** | **P0** |
| 降低分辨率(416) | -4ms | 65-70 | P1 |
| 零拷贝相机 | -1ms | 70-75 | P2 |
| 流水线并行 | -1.5ms | 75-80 | P2 |
| 编译器优化 | -0.5ms | 80-85 | P3 |

**目标: 60-80 FPS** 可通过RGA + 模型优化实现

---

## 🔧 快速开始

### 步骤1: 编译启用RGA

```bash
cd /path/to/rk3576_rknn_targeting/cpp_fast_targeting

# 方法A: 使用脚本
bash build_rga.sh

# 方法B: 手动编译
rm -rf build && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DRKFAST_USE_RGA=ON
make -j$(nproc)
```

### 步骤2: 安装RGA库 (如果缺失)

```bash
# 在RK3576板上
sudo apt update
sudo apt install librga-dev librga2

# 检查安装
ls /usr/include/rga/
ls /usr/lib/librga.so
```

### 步骤3: 运行并验证

```bash
cd build
./rkfast_target \
  --model ../models/best_rk3576_int8.rknn \
  --camera /dev/video73 \
  --width 640 --height 480 --fps 60 \
  --fourcc MJPG \
  --no-window \
  --stats-interval 2.0

# 预期输出:
# RGA hardware acceleration enabled
# RGA hardware acceleration initialized
# stats frame_fps=50.2 infer_fps=58.1 pre=1.5ms rknn=7.4ms ...
#       ^^^^^^^^ 提升到50+    ^^^^^^ RGA加速!
```

### 步骤4: 对比测试

```bash
# 禁用RGA (对比)
cmake .. -DRKFAST_USE_RGA=OFF
make -j$(nproc)
./rkfast_target ...
# 应该看到: pre=9.0ms, frame_fps=27.5

# 启用RGA
cmake .. -DRKFAST_USE_RGA=ON
make -j$(nproc)
./rkfast_target ...
# 应该看到: pre=1.5ms, frame_fps=50+
```

---

## 🐛 故障排除

### 问题1: CMake找不到RGA

```
CMake Warning: RGA requested but not found, disabling RGA support
```

**解决:**
```bash
sudo apt install librga-dev librga2

# 或手动指定路径
cmake .. \
  -DRGA_INCLUDE_DIR=/usr/include \
  -DRGA_LIB=/usr/lib/librga.so
```

### 问题2: 运行时RGA初始化失败

```
RGA init failed (RGA init failed: -1), using OpenCV fallback
```

**解决:**
```bash
# 检查RGA设备权限
ls -l /dev/rga
sudo chmod 666 /dev/rga

# 添加用户到video组
sudo usermod -aG video $USER
newgrp video

# 检查内核模块
dmesg | grep rga
lsmod | grep rga
```

### 问题3: RGA编译成功但预处理仍然慢

```
stats ... pre=8.5ms  # 没有加速!
```

**检查:**
```bash
# 确认RGA初始化消息
./rkfast_target ... 2>&1 | grep RGA
# 应该看到:
# RGA hardware acceleration enabled
# RGA hardware acceleration initialized

# 如果看到 "using OpenCV fallback",检查:
# 1. /dev/rga 权限
# 2. librga.so 是否正确链接: ldd ./rkfast_target | grep rga
# 3. 内核RGA驱动: dmesg | grep rga
```

---

## 📚 其他优化资源

### MCP工具使用

如果你有MCP (Model Context Protocol) 服务器,可以:

```bash
# 使用filesystem MCP读取性能日志
# 使用brave-search MCP查找RK3576优化文档
# 使用github MCP查找Rockchip示例代码
```

### Rockchip官方文档

- [RKNN Toolkit2 文档](https://github.com/rockchip-linux/rknn-toolkit2)
- [RGA API参考](https://github.com/airockchip/librga)
- [RK3576 TRM](https://rockchip.fr/RK3576%20TRM)

### 社区资源

- Rockchip开发者论坛
- RK3588/RK3576优化案例 (架构相似)
- RKNN模型优化工具

---

## ✅ 总结

**已完成:**
1. ✅ RGA硬件加速模块实现
2. ✅ CMake构建系统集成
3. ✅ 自动降级到OpenCV
4. ✅ 编译优化标志
5. ✅ 构建脚本和文档

**立即行动:**
```bash
cd cpp_fast_targeting
bash build_rga.sh
./build/rkfast_target --model ../models/best_rk3576_int8.rknn \
  --camera /dev/video73 --no-window
```

**预期结果:**
- 预处理: 9.0ms → 1.5ms
- 帧率: 27.5 FPS → **48-55 FPS**
- 如需更高帧率,考虑降低模型分辨率到416x416

**最终目标: 60-80 FPS** 🎯
