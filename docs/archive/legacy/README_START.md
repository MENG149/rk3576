# RK3576 RKNN Targeting - Quick Start Guide

## 🚀 One-Command Start (Recommended)

```bash
cd ~/rk3576_rknn_targeting
bash quick_start.sh
```

This will:
- ✅ Enable performance mode (CPU/GPU/NPU)
- ✅ Build with RGA acceleration (if needed)
- ✅ Start RKNN targeting program

---

## 📋 Available Scripts

### 1. `quick_start.sh` - Fast Launch
```bash
bash quick_start.sh
```
- Minimal output
- Auto-build if needed
- Best for daily use

### 2. `start_performance_mode.sh` - Detailed Launch
```bash
bash start_performance_mode.sh
```
- Detailed status output
- Hardware checks
- Configuration display
- Best for debugging

### 3. `tools/rk3576_performance.sh` - Performance Mode Only
```bash
sudo bash tools/rk3576_performance.sh
```
- Sets CPU/GPU/NPU to max performance
- Run once per boot
- Can be used standalone

### 4. `cpp_fast_targeting/build_rga.sh` - Build Only
```bash
cd cpp_fast_targeting
bash build_rga.sh
```
- Compiles with RGA acceleration
- Run after code changes

---

## 📊 Expected Output

### Successful Start:

```
======================================
  RK3576 RKNN 快速启动
======================================

[1/3] 启动性能模式...
  ✓ CPU: performance
  ✓ GPU: performance
  ✓ NPU: performance

[2/3] 检查程序...
✓ 程序已就绪

[3/3] 启动程序...
======================================

RGA hardware acceleration enabled
RKNN core_mask=01
RGA hardware acceleration initialized
running; press q in the debug window to exit

stats frame_fps=50.2 infer_fps=58.1 pre=1.5ms rknn=7.4ms ...
```

### Key Indicators:
- ✅ `RGA hardware acceleration initialized` - RGA working
- ✅ `frame_fps > 45` - Good performance
- ✅ `pre < 2ms` - Hardware acceleration active
- ✅ `state=LOCKED` - Target tracking

---

## ⚙️ Configuration

### Modify Camera Device

Edit `quick_start.sh` or `start_performance_mode.sh`:

```bash
CAMERA_DEV="/dev/video0"  # Change to your device
```

Find available cameras:
```bash
ls -l /dev/video*
v4l2-ctl --list-devices
```

### Modify NPU Cores

```bash
CORE_MASK="0"    # Single core
CORE_MASK="01"   # Dual core (recommended)
CORE_MASK="012"  # Triple core
CORE_MASK="auto" # Auto scheduling
```

### Modify Resolution

```bash
WIDTH=640
HEIGHT=480
```

---

## 🐛 Troubleshooting

### Permission Denied
```bash
sudo bash quick_start.sh
```

### RGA Not Working (pre=9ms)
```bash
# Check RGA device
ls -l /dev/rga
sudo chmod 666 /dev/rga

# Rebuild
cd cpp_fast_targeting
rm -rf build
bash build_rga.sh
```

### Camera Not Found
```bash
# List cameras
ls -l /dev/video*

# Edit script and change CAMERA_DEV
```

### Model Not Found
```bash
# Check model path
ls -l models/

# Edit script and change MODEL_PATH
```

---

## 📈 Performance Targets

| Metric | Target | Actual |
|--------|--------|--------|
| Frame FPS | > 45 | 48-55 |
| Preprocessing | < 2ms | 1.5ms |
| RKNN Inference | < 10ms | 7.6ms |
| Total Latency | < 12ms | 9.5ms |

---

## 📚 Documentation

- **中文启动说明**: `启动说明.md`
- **性能优化总结**: `PERFORMANCE_OPTIMIZATION_SUMMARY.md`
- **完整优化指南**: `cpp_fast_targeting/OPTIMIZATION_GUIDE.md`
- **RGA使用说明**: `cpp_fast_targeting/README_RGA.md`

---

## ✅ Success Checklist

Before competition:

- [ ] Performance mode enabled
- [ ] RGA acceleration working (`pre < 2ms`)
- [ ] Frame rate > 45 FPS
- [ ] Camera working
- [ ] Target tracking stable
- [ ] Serial output correct

---

## 🎯 Quick Reference

```bash
# Start everything
bash quick_start.sh

# Check status
top -bn1 | grep rkfast

# Stop program
pkill rkfast_target

# Rebuild
cd cpp_fast_targeting && bash build_rga.sh

# View logs
journalctl -f | grep rkfast
```

**Good luck in the competition!** 🚀
