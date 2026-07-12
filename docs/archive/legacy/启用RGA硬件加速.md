# 启用RGA硬件加速 - 完整指南

## 当前状态

✅ **RGA硬件存在:**
```bash
user@localhost:~/rk3576_rknn_targeting$ ls -l /sys/bus/platform/devices/ | grep rga
lrwxrwxrwx 1 root root 0 Mar 24 21:45 27920000.rga -> ../../../devices/platform/27920000.rga
lrwxrwxrwx 1 root root 0 Mar 24 21:45 27930000.rga -> ../../../devices/platform/27930000.rga
```

❌ **问题:** 设备节点 `/dev/rga` 可能不存在或RGA API不兼容

---

## 解决步骤

### 步骤1: 运行RGA诊断

```bash
cd ~/rk3576_rknn_targeting
bash test_rga.sh
```

这会检查:
- RGA设备节点
- RGA库文件
- 设备权限
- 自动创建缺失的设备节点

### 步骤2: 手动创建RGA设备节点(如果需要)

```bash
# 检查设备号
cat /sys/devices/platform/27920000.rga/dev
# 输出类似: 241:0

# 创建设备节点 (假设major=241, minor=0)
sudo mknod /dev/rga c 241 0
sudo chmod 666 /dev/rga

# 验证
ls -l /dev/rga
```

### 步骤3: 检查RGA库版本

```bash
# 查找RGA库
find /usr -name "librga*"

# 检查符号链接
ldconfig -p | grep rga

# 查看RGA头文件API版本
grep -r "imresize_t" /usr/include/rga/
```

### 步骤4: 测试RGA API

创建测试程序:

```bash
cat > /tmp/test_rga_simple.cpp << 'EOF'
#include <iostream>
#include <cstring>

// 尝试包含RGA头文件
#ifdef __has_include
#  if __has_include(<rga/RgaApi.h>)
#    include <rga/RgaApi.h>
#    include <rga/im2d.h>
#    define HAS_RGA 1
#  endif
#endif

int main() {
#ifdef HAS_RGA
    std::cout << "Testing RGA..." << std::endl;
    
    // 测试初始化
    int ret = c_RkRgaInit();
    std::cout << "c_RkRgaInit() = " << ret << std::endl;
    
    if (ret == 0 || ret == IM_STATUS_SUCCESS) {
        std::cout << "✓ RGA OK!" << std::endl;
        
        // 查询版本
        const char* ver = querystring(RGA_VERSION);
        if (ver) std::cout << "Version: " << ver << std::endl;
        
        const char* vendor = querystring(RGA_VENDOR);
        if (vendor) std::cout << "Vendor: " << vendor << std::endl;
        
        c_RkRgaDeInit();
        return 0;
    } else {
        std::cout << "✗ RGA init failed" << std::endl;
        return 1;
    }
#else
    std::cout << "RGA headers not found" << std::endl;
    return 1;
#endif
}
EOF

# 编译
g++ /tmp/test_rga_simple.cpp -o /tmp/test_rga -lrga -I/usr/include

# 运行
/tmp/test_rga
```

### 步骤5: 根据测试结果修复

#### 情况A: RGA库不存在

```bash
# 安装RGA库
sudo apt update
sudo apt install librga-dev librga2

# 或从Rockchip SDK安装
# 下载 librga.so 和头文件
sudo cp librga.so /usr/lib/
sudo cp -r rga/ /usr/include/
sudo ldconfig
```

#### 情况B: RGA API版本不匹配

检查你的RGA API版本:

```bash
# 查看imresize_t函数签名
grep -A 3 "imresize_t" /usr/include/rga/im2d_single.h
```

可能的签名:
```cpp
// API v1
IM_STATUS imresize_t(rga_buffer_t src, rga_buffer_t dst, 
                     double fx, double fy, int interpolation);

// API v2  
IM_STATUS imresize_t(rga_buffer_t src, rga_buffer_t dst,
                     double fx, double fy, int interpolation, int sync);
```

根据实际API修改 `src/rga_preprocess.cpp`

#### 情况C: 设备权限问题

```bash
# 添加用户到正确的组
sudo usermod -aG video user
sudo usermod -aG render user

# 设置udev规则
sudo tee /etc/udev/rules.d/99-rga.rules << EOF
KERNEL=="rga", MODE="0666", GROUP="video"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

# 重新登录
newgrp video
```

---

## 快速修复方案

### 方案1: 使用DRM/DRI接口(如果RGA不工作)

某些系统RGA通过DRM暴露:

```bash
ls -l /dev/dri/
# 如果看到 renderD128 等设备,RGA可能通过DRI工作
```

### 方案2: 禁用RGA,使用OpenCV(当前可用)

```bash
cd ~/rk3576_rknn_targeting/cpp_fast_targeting
rm -rf build
mkdir build && cd build

# 禁用RGA编译
cmake .. -DCMAKE_BUILD_TYPE=Release -DRKFAST_USE_RGA=OFF
make -j$(nproc)

# 运行测试
./rkfast_target \
  --model ../../models/best_yolo26n_512_rk3576_int8.rknn \
  --camera /dev/video73 \
  --no-window
```

**性能:** 27-30 FPS (足够测试)

### 方案3: 使用OpenCL加速(替代RGA)

如果RGA不工作,可以尝试OpenCL:

```bash
# 检查OpenCL
clinfo

# OpenCV可以使用OpenCL加速
export OPENCV_OPENCL_DEVICE=:GPU:0
```

---

## 预期性能对比

| 方案 | 预处理 | 帧率 | 难度 |
|------|--------|------|------|
| **RGA硬件** | 1.5ms | 48-55 FPS | 需要调试 |
| **OpenCV CPU** | 9.0ms | 27-30 FPS | ✅ 当前可用 |
| **OpenCV+OpenCL** | 4-5ms | 35-40 FPS | 中等 |
| **降低分辨率(512→416)** | 6ms | 35-40 FPS | 简单 |

---

## 立即测试极限性能

即使RGA不工作,也可以测试其他优化:

```bash
cd ~/rk3576_rknn_targeting/cpp_fast_targeting/build

# 测试1: 使用3个NPU核心
./rkfast_target \
  --model ../../models/best_yolo26n_512_rk3576_int8.rknn \
  --camera /dev/video73 \
  --core-mask 012 \
  --no-window

# 测试2: 降低相机分辨率
./rkfast_target \
  --model ../../models/best_yolo26n_512_rk3576_int8.rknn \
  --camera /dev/video73 \
  --width 320 --height 240 \
  --core-mask 01 \
  --no-window

# 测试3: 调整检测间隔
./rkfast_target \
  --model ../../models/best_yolo26n_512_rk3576_int8.rknn \
  --camera /dev/video73 \
  --core-mask 01 \
  --detect-interval-locked 5 \
  --no-window
```

---

## 下一步

1. **立即:** 运行 `bash test_rga.sh` 诊断
2. **然后:** 根据诊断结果选择修复方案
3. **或者:** 先用OpenCV模式测试功能,稍后优化RGA

**命令:**
```bash
cd ~/rk3576_rknn_targeting
bash test_rga.sh
```

把诊断结果发给我,我帮你分析!
