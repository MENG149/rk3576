# RK3576 RKNN Targeting - Performance Optimization Guide

## Current Performance Analysis

**Baseline Stats:**
```
frame_fps=27.5 infer_fps=57.3
pre=9.0ms rknn=7.6ms post=0.2ms total=17.1ms
```

**Bottleneck:** Preprocessing (9ms) is the main bottleneck, consuming 53% of total time.

---

## 🚀 Optimization 1: RGA Hardware Acceleration (CRITICAL)

### What is RGA?
Rockchip RGA (Raster Graphic Acceleration) is a 2D hardware accelerator that can perform:
- Image resize (bilinear/nearest)
- Color space conversion (BGR↔RGB)
- Format conversion
- Rotation/flip

### Expected Improvement
- **Before:** 9.0ms preprocessing (OpenCV CPU)
- **After:** 1-2ms preprocessing (RGA hardware)
- **Gain:** ~7ms saved → **frame_fps: 27.5 → 45-50 FPS**

### Build with RGA Support

```bash
cd cpp_fast_targeting
mkdir -p build && cd build

# Enable RGA (default ON)
cmake .. -DCMAKE_BUILD_TYPE=Release -DRKFAST_USE_RGA=ON

# Or disable if RGA libs not available
# cmake .. -DCMAKE_BUILD_TYPE=Release -DRKFAST_USE_RGA=OFF

make -j$(nproc)
```

### Install RGA Libraries (if missing)

```bash
# On RK3576 board
sudo apt update
sudo apt install librga-dev librga2

# Or manually install from Rockchip SDK
# Headers: /usr/include/rga/
# Library: /usr/lib/librga.so
```

### Verify RGA at Runtime

When you run the program, you should see:
```
RGA hardware acceleration enabled
RGA hardware acceleration initialized
```

If you see:
```
RGA init failed (...), using OpenCV fallback
```
Then RGA is not available and OpenCV fallback is used.

---

## 🔧 Optimization 2: Zero-Copy Camera Capture

### Current Issue
`camera_source.cpp:106` copies frame with `std::move`, but OpenCV may still allocate.

### Solution: Use V4L2 MMAP + DMA buffers

```cpp
// In camera_source.cpp, use:
cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);  // Already done ✓
cap_.set(cv::CAP_PROP_MODE, cv::CAP_MODE_GRAY);  // If grayscale input

// Better: Direct V4L2 with MMAP
// Replace cv::VideoCapture with libv4l2 + mmap for true zero-copy
```

**Expected gain:** 0.5-1ms

---

## ⚡ Optimization 3: RKNN Multi-Core Scheduling

### Current Setup
You're using multi-core (infer_fps=57.3 > frame_fps=27.5), which is good.

### Fine-tune Core Mask

RK3576 has 3 NPU cores. Test different configurations:

```bash
# Single core (baseline)
./rkfast_target --core-mask 0

# Dual core (current, likely)
./rkfast_target --core-mask 01

# Triple core (may help if model is large)
./rkfast_target --core-mask 012

# Auto scheduling
./rkfast_target --core-mask auto
```

**Current:** rknn=7.6ms suggests you're already using multi-core effectively.

---

## 🎯 Optimization 4: Reduce Inference Resolution

### Trade Accuracy for Speed

If your model is 640x640, try:
- **480x480:** ~40% faster inference
- **416x416:** ~50% faster inference
- **320x320:** ~70% faster inference

Re-export RKNN model at lower resolution:

```python
# In RKNN conversion script
rknn.config(target_platform='rk3576',
            quantized_dtype='asymmetric_quantized-8',
            optimization_level=3,
            input_size_list=[[1, 3, 416, 416]])  # Lower resolution
```

**Expected gain:** 3-5ms (rknn: 7.6ms → 3-4ms)

---

## 🧹 Optimization 5: Disable Unnecessary Features

### Remove Debug Overhead

```bash
# Disable window rendering (already done with --no-window)
./rkfast_target --no-window

# Reduce stats interval
./rkfast_target --stats-interval 5.0  # Print every 5s instead of 1s

# Disable rect refiner if not needed
# Edit main.cpp:148-154 to skip refiner.Refine()
```

**Expected gain:** 0.2-0.5ms

---

## 🔥 Optimization 6: Compiler Optimizations

### Already Applied (in CMakeLists.txt)
```cmake
-O3                  # Maximum optimization
-ffast-math          # Fast floating-point
-fno-math-errno      # Skip errno checks
-fno-trapping-math   # No FP exception traps
```

### Additional Flags (optional)

Edit `CMakeLists.txt:129`:

```cmake
target_compile_options(
  rkfast_target
  PRIVATE
    -O3
    -ffast-math
    -fno-math-errno
    -fno-trapping-math
    -march=native          # Use all CPU features
    -mtune=cortex-a76      # Tune for RK3576 cores
    -flto                  # Link-time optimization
)
```

And add to linking:
```cmake
target_link_options(rkfast_target PRIVATE -flto)
```

**Expected gain:** 0.5-1ms

---

## 📊 Optimization 7: Profiling Hotspots

### Use perf to find bottlenecks

```bash
# On RK3576 board
sudo perf record -g ./rkfast_target --no-window
# Run for 30 seconds, then Ctrl+C
sudo perf report

# Look for:
# - cv::resize
# - cv::cvtColor
# - rknn_run
# - Mutex locks
```

---

## 🎬 Optimization 8: Pipeline Parallelism

### Current Architecture
```
Camera Thread → Main Thread → RKNN Worker Thread
                ↓
              Tracker/Refiner
```

### Improved: Separate Refiner Thread

```cpp
// Move rect_refiner to its own thread
// So main loop becomes:
while (true) {
  frame = camera.ReadLatest();
  result = worker.LatestResult();  // Non-blocking
  refiner_worker.Submit(frame);    // Async refine
  tracker.Update(...);
}
```

**Expected gain:** 1-2ms (overlap refiner with RKNN)

---

## 📈 Expected Final Performance

| Optimization | Time Saved | Cumulative FPS |
|--------------|------------|----------------|
| Baseline | - | 27.5 FPS |
| + RGA acceleration | -7ms | **45-50 FPS** |
| + Zero-copy camera | -1ms | **52-55 FPS** |
| + Lower resolution (416x416) | -4ms | **65-70 FPS** |
| + Compiler flags | -0.5ms | **70-75 FPS** |
| + Pipeline parallelism | -1ms | **75-80 FPS** |

**Target: 60-80 FPS** is achievable with RGA + model optimization.

---

## 🚦 Quick Start: Enable RGA Now

```bash
cd /path/to/cpp_fast_targeting
rm -rf build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DRKFAST_USE_RGA=ON
make -j4

# Run and verify RGA is active
./rkfast_target --model ../models/best_rk3576_int8.rknn \
  --camera /dev/video73 --no-window

# You should see:
# "RGA hardware acceleration enabled"
# "RGA hardware acceleration initialized"
# pre=1.5ms (instead of 9.0ms)
```

---

## 🔍 Troubleshooting

### RGA Not Found During Build
```
CMake Warning: RGA requested but not found, disabling RGA support
```

**Fix:**
```bash
sudo apt install librga-dev librga2
# Or set manually:
cmake .. -DRGA_INCLUDE_DIR=/usr/include -DRGA_LIB=/usr/lib/librga.so
```

### RGA Init Failed at Runtime
```
RGA init failed (RGA init failed: -1), using OpenCV fallback
```

**Fix:**
```bash
# Check RGA device permissions
ls -l /dev/rga
sudo chmod 666 /dev/rga

# Or add user to video group
sudo usermod -aG video $USER
```

### Still Slow After RGA

Check stats output:
```
pre=8.5ms  # RGA not actually running!
```

Look for:
```
RGA hardware acceleration initialized  # Should appear
```

If missing, RGA fallback to OpenCV. Check `dmesg | grep rga` for kernel errors.

---

## 📝 Summary

**Priority 1 (Do Now):**
1. ✅ Enable RGA hardware acceleration → **+20 FPS**
2. Test different RKNN core masks
3. Disable debug window

**Priority 2 (If Needed):**
4. Lower model resolution (416x416)
5. Add compiler optimizations
6. Profile with perf

**Priority 3 (Advanced):**
7. Zero-copy V4L2 camera
8. Pipeline parallelism
9. Custom RKNN model optimization

**Expected Result:** 60-80 FPS with RGA + optimizations.
