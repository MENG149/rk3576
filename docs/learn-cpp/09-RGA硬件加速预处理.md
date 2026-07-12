# 09 -- RGA 硬件加速预处理：让缩放飞起来

## 1. RGA 是什么

上一章讲到预处理要做三件事：**缩放、letterbox 填充、BGR 转 RGB**。如果用 OpenCV（也就是 CPU 算），每帧大概要花 1.5~3ms。

但 RK3576 芯片上有一个专门的硬件模块叫 **RGA（Raster Graphics Accelerator）**，中文叫"光栅图形加速器"。它是专门设计来做图像处理的硬件——就像 NPU 是专门做神经网络计算的硬件一样。

RGA 能干这些事：
- **尺寸缩放**（resize）：把 640x480 缩成 640x640
- **色彩空间转换**（csc，color space conversion）：BGR 转 RGB、YUV 转 RGB 等
- **格式转换**：比如 YUV420 转 BGR888
- **旋转/翻转**：90 度旋转、镜像翻转
- **裁剪 + 合成**：把多张图拼成一张

关键优势：**它在硬件里做这些事，不消耗 CPU**。CPU 可以同时干别的活儿。

你可以把 RGA 理解成显卡里的"2D 引擎"——不是画 3D 游戏的那种 GPU 核心，而是专门处理 2D 图像的协处理器。

---

## 2. 预处理流程图：RGA 负责什么，CPU 负责什么

```
摄像头输出：BGR 640x480（HWC交错排列）
    │
    ▼
┌─────────────────────────────────────┐
│  RGA 硬件（一步完成）：              │
│  - resize 从 640x480 到 640x480     │
│    （本例中 scale=1.0，不需要缩放）  │
│  - BGR → RGB 色彩转换               │
│  - letterbox：画面居中，上下填灰色   │
│  - 输出到 input_rgb_ 缓冲区          │
└─────────────────────────────────────┘
    │
    │ 输出：RGB 640x640（HWC交错排列）
    ▼
┌─────────────────────────────────────┐
│  CPU（仅做拆分通道）：               │
│  cv::split() 把 HWC 拆成三个平面     │
│  R平面 G平面 B平面 → input_chw_     │
└─────────────────────────────────────┘
    │
    ▼
送入 NPU（NCHW 格式）
```

RGA 做了最重的活儿——resize、色彩转换、letterbox 填充。CPU 只做了一件轻活儿——拆通道。如果模型用的是 NHWC 格式，连拆通道都不用，RGA 直接输出就是 NPU 吃进去的格式。

---

## 3. RgaPreprocessor 类结构

```cpp
class RgaPreprocessor {
 public:
  bool Init(int input_width, int input_height, std::string* error);
  void Release();
  bool Process(const cv::Mat& bgr_frame, cv::Mat* rgb_output,
               float* scale, int* pad_x, int* pad_y,
               std::string* error);
  bool IsAvailable() const { return available_; }  // RGA 能用吗？

 private:
  bool available_ = false;          // RGA 是否可用
  int input_width_ = 0;             // 模型输入宽度（640）
  int input_height_ = 0;            // 模型输入高度（640）
  cv::Mat letterbox_buffer_;       // 预分配的输出缓冲区

#ifdef RKFAST_USE_RGA
  rga_buffer_t src_buf_{};         // RGA 源缓冲区描述符
  rga_buffer_t dst_buf_{};         // RGA 目标缓冲区描述符
#endif
};
```

三个公开方法对应完整生命周期：

```
Init()  →  Process() × N  →  Release()
 开机       每帧调用           关机
```

`IsAvailable()` 让调用方可以判断 RGA 是否真的能工作。如果 `Init()` 失败了，`available_` 保持 `false`，后续调用 `Process()` 会直接返回失败——从而触发 OpenCV 降级路径。

---

## 4. Init：初始化 RGA 设备

```cpp
bool RgaPreprocessor::Init(int input_width, int input_height,
                           std::string* error) {
#ifdef RKFAST_USE_RGA
  input_width_ = input_width;
  input_height_ = input_height;

  // ① 预分配 letterbox 输出缓冲区
  letterbox_buffer_ = cv::Mat(input_height_, input_width_, CV_8UC3,
                              cv::Scalar(114, 114, 114));

  // ② 初始化 RGA 设备
  int ret = c_RkRgaInit();
  if (ret != IM_STATUS_SUCCESS && ret != 0) {  // 注意：有些版本返回0表示成功
    if (error != nullptr) {
      *error = "RGA init failed: " + std::to_string(ret);
    }
    available_ = false;
    return false;
  }

  available_ = true;
  std::cout << "RGA hardware acceleration enabled\n";
  return true;
#else
  // 编译时没定义 RKFAST_USE_RGA，直接返回失败
  if (error != nullptr) {
    *error = "RGA support not compiled (define RKFAST_USE_RGA)";
  }
  available_ = false;
  return false;
#endif
}
```

### ① 为什么提前分配 letterbox_buffer_？

每帧预处理时，RGA 的输出需要一块固定的内存。如果每帧都 `new` 一块内存再 `delete`，频繁分配/释放会拖慢速度。所以 **Init 时一次性分配好**，之后每帧复用这块缓冲区。

`cv::Scalar(114, 114, 114)` 初始化为灰色——和 YOLO 训练时的填充色一致。这样每帧只需要把灰色"覆盖"成实际画面，不需要重新填充灰色。

### ② `c_RkRgaInit()` 返回值的坑

注释里写了：`Some RGA versions return 0 for success instead of IM_STATUS_SUCCESS`。这是因为不同版本的 RGA 库行为不一致——有的用 `IM_STATUS_SUCCESS`（=0）表示成功，有的用其他值。所以代码判断 `ret != IM_STATUS_SUCCESS && ret != 0` 才认为失败。

这体现了嵌入式开发的现实：**板子上的库版本千差万别，代码要兼容各种情况**。

---

## 5. Process：核心预处理

```cpp
bool RgaPreprocessor::Process(const cv::Mat& bgr_frame, cv::Mat* rgb_output,
                              float* scale, int* pad_x, int* pad_y,
                              std::string* error) {
#ifdef RKFAST_USE_RGA
  if (!available_ || bgr_frame.empty()) return false;

  // ① 计算 letterbox 参数
  const float scale_val = std::min(
      float(input_width_)  / frame_w,     // 640/640 = 1.0
      float(input_height_) / frame_h);    // 640/480 ≈ 1.33
  const int new_w = round(frame_w * scale_val);  // 缩放后宽度
  const int new_h = round(frame_h * scale_val);  // 缩放后高度
  const int pad_x_val = (input_width_  - new_w) / 2;  // 左右padding
  const int pad_y_val = (input_height_ - new_h) / 2;  // 上下padding

  // ② 把输出缓冲区刷成灰色（每次都要，因为上次覆盖了）
  letterbox_buffer_.setTo(cv::Scalar(114, 114, 114));

  // ③ 包装 RGA 源缓冲区（指向摄像头帧）
  rga_buffer_t src = wrapbuffer_virtualaddr(
      bgr_frame.data,             // 源数据指针
      frame_w, frame_h,           // 宽高
      RK_FORMAT_BGR_888,          // 色彩格式：BGR 三个字节
      frame_w,                    // wstride（行步长）
      frame_h);                   // hstride（列步长）

  // ④ 包装 RGA 目标缓冲区（指向 letterbox 的有效区域）
  rga_buffer_t dst = wrapbuffer_virtualaddr(
      letterbox_buffer_(cv::Rect(pad_x_val, pad_y_val, new_w, new_h)).data,
      new_w, new_h,
      RK_FORMAT_RGB_888,          // 目标格式：RGB！
      new_w, new_h);

  // ⑤ 硬件 resize + BGR2RGB（一步完成！）
  IM_STATUS status = imresize_t(src, dst, scale_val, scale_val,
                                INTER_LINEAR, 1);  // 1 = 同步等待
  if (status != IM_STATUS_SUCCESS) {
    // 降级：只用 RGA 做色彩转换
    status = imcvtcolor(src, dst, src.format, dst.format);
    // ...
  }
#endif
}
```

### ③④ `wrapbuffer_virtualaddr` 是什么？

这是 RGA 库提供的函数，作用是把一块**普通内存**包装成 RGA 认识的"缓冲区描述符"。

```cpp
rga_buffer_t src = wrapbuffer_virtualaddr(
    数据指针,     // void* 指向实际像素数据
    宽度, 高度,   // 图像的尺寸
    色彩格式,     // 告诉RGA数据是什么格式（BGR888、RGB888、YUV420...）
    wstride,      // 一行占多少字节（通常 = 宽度 × 每像素字节数）
    hstride);     // 一列占多少像素（通常 = 高度）
```

wstride 和 hstride 是什么？正常情况下 `wstride = 宽度`, `hstride = 高度`。但如果图像是 YUV420 这种特殊格式，或者内存里有对齐 padding，stride 可能和尺寸不一致。你项目里用的是 BGR888/RGB888，每像素 3 字节，每行连续存放，所以 stride = width。

### ⑤ `imresize_t` —— 一个函数完成缩放 + 格式转换

```cpp
imresize_t(src, dst,            // 源和目标缓冲区
           scale_val, scale_val,// x和y方向缩放比例
           INTER_LINEAR,        // 插值算法：双线性
           1);                  // sync=1：等硬件干完再返回
```

这个函数叫 `imresize_t`（带 `_t` 后缀的版本），它不仅能 resize，还能**同时做色彩格式转换**。因为源格式是 `RK_FORMAT_BGR_888`，目标格式是 `RK_FORMAT_RGB_888`，RGA 在缩放的时候顺手就把 BGR 转成 RGB 了。

最后一个参数 `1` 表示同步模式——函数会阻塞，等 RGA 硬件干完活才返回。如果用 `0`（异步模式），函数立即返回但数据可能还没写完，需要手动等 hardware fence——麻烦且容易出 bug。同步模式简单可靠。

插值算法 `INTER_LINEAR`（双线性插值）是缩放质量和速度之间的平衡选择：
- `INTER_NEAREST`：最快，但锯齿明显
- `INTER_LINEAR`：折中，速度快且质量够用
- `INTER_CUBIC`：最好，但慢

你的场景里 640x480 → 640x640 基本不需要缩放（比例接近 1:1），所以 `INTER_LINEAR` 足够了。

### 降级策略：如果 imresize_t 失败，用 imcvtcolor

```cpp
if (status != IM_STATUS_SUCCESS) {
  // 只用 RGA 做色彩转换（不做缩放）
  status = imcvtcolor(src, dst, src.format, dst.format);
}
```

`imcvtcolor` 只做色彩空间转换，不做缩放。这是更深一层的降级——至少 RGA 还能帮忙把 BGR 转成 RGB。

---

## 6. 为什么 RGA 比 OpenCV 快 6 倍？

这不是一个精确数字，而是一个量级概念。在 RK3576 上，用 RGA 做预处理大约 0.3~0.5ms，用 OpenCV 大约 1.5~4ms。差异来自几点：

### 6.1 专用硬件 vs 通用 CPU

OpenCV 在 CPU 上跑，每处理一个像素要执行一系列指令（取数、算新坐标、插值、存结果）。CPU 虽然是多核的，但要跟其他线程抢时间片。

RGA 是专用硬件——它内部有专门做缩放的"管线"（pipeline），数据流水一样哗哗地流过去，缩放和格式转换在硬件里并行完成。就像快递分拣：人工（CPU）一个个拿包裹，传送带（RGA）自动分流。

### 6.2 DMA 搬运 vs CPU 搬运

RGA 用 **DMA（直接内存访问）** 搬数据。DMA 的意思是：数据直接在内存和外设之间传输，不经过 CPU。

```
CPU方式：内存 → CPU寄存器 → 处理 → CPU寄存器 → 内存（CPU全程参与）
DMA方式：内存 → RGA硬件 → 内存（CPU完全不管）
```

CPU 在 RGA 干活时可以去做别的事——比如准备下一帧数据、更新跟踪状态、发串口指令。

### 6.3 零拷贝：数据和色彩转换同时完成

看这行代码：

```cpp
rga_buffer_t dst = wrapbuffer_virtualaddr(
    letterbox_buffer_(cv::Rect(pad_x, pad_y, new_w, new_h)).data,
    ...);
```

目标缓冲区指向的是 `letterbox_buffer_` 的一个 ROI（感兴趣区域）。RGA 直接把缩放后的画面写到 letterbox 的"有效画面区域"里，不需要先写到临时缓冲区再拷过去——**零额外拷贝**。

### 6.4 实际性能对比

在你的场景（640x480 → 640x640，BGR→RGB，letterbox）下：

| 方式 | 耗时 | 说明 |
|------|------|------|
| RGA 硬件 | ~0.3ms | resize + BGR2RGB 一步完成 |
| OpenCV (优化) | ~1.5ms | resize + cvtColor 两步，CPU |
| OpenCV (未优化) | ~3ms+ | 未限制线程数时更慢 |

你的 `config.hpp` 里设置了 `opencv_threads = 1`，限制了 OpenCV 只用单线程——避免 OpenCV 内部多线程调度和你的三线程架构竞争。RGA 不受这个限制，因为它不占用 CPU 线程。

---

## 7. `#ifdef RKFAST_USE_RGA`：条件编译详解

你注意到 `rga_preprocess.hpp` 和 `.cpp` 里到处都是 `#ifdef RKFAST_USE_RGA`。这是什么？

```cpp
// rga_preprocess.hpp
#ifdef RKFAST_USE_RGA
#include <rga/RgaApi.h>      // 只有定义了宏才 include RGA 头文件
#include <rga/im2d.h>
#endif
```

**条件编译**的意思是：编译时根据有没有定义某个宏，决定哪些代码参与编译。

**为什么需要这个？**

RGA 库（`librga.so`）只在 Rockchip 的板子上才有。如果你在 PC 上编译这个项目（比如用 VS Code 编辑代码，或者跑单元测试），PC 上没有 `librga.so`，也没有 `/dev/rga` 设备——编译会报错"找不到 rga/RgaApi.h"。

用条件编译后：
- **板子上编译**：`cmake -DRKFAST_USE_RGA=ON`，定义宏，RGA 代码参与编译
- **PC 上编译**：不定义宏，RGA 代码被完全跳过

CMakeLists.txt 里对应的逻辑大概是：

```cmake
if(RKFAST_USE_RGA)
  target_compile_definitions(rkfast PRIVATE RKFAST_USE_RGA)
  target_link_libraries(rkfast rga)
endif()
```

---

## 8. 自动降级：RGA 失败了怎么办？

嵌入式系统不像 PC——硬件不一定每次都可靠。`/dev/rga` 设备可能因为内核驱动版本不对而不存在，RGA 库版本和板子不匹配也可能导致初始化失败。

你的代码设计了一条"优雅降级"路径：

```
尝试 RGA Init
  │
  ├─ 成功 → available_ = true  → 每帧用 RGA 预处理
  │
  └─ 失败 → available_ = false → 每帧用 OpenCV 预处理
             打印：RGA init failed (xxx), using OpenCV fallback
```

在 `RknnDetector::Open()` 里：

```cpp
std::string rga_error;
if (rga_preprocessor_.Init(input_width_, input_height_, &rga_error)) {
  std::cout << "RGA hardware acceleration initialized\n";
} else {
  std::cout << "RGA init failed (" << rga_error << "), using OpenCV fallback\n";
  // 不 return false！继续运行，用 OpenCV
}
```

注意：RGA 失败**不导致程序退出**。程序继续运行，只是预处理走 OpenCV 路径，慢一点而已。这种"尽力而为，降级运行"的设计哲学在嵌入式系统里非常重要——不能因为一个优化功能挂了就让整个系统瘫痪。

在 `Preprocess()` 里也是这样：

```cpp
if (rga_preprocessor_.IsAvailable()) {
  if (rga_preprocessor_.Process(...)) {
    // RGA 成功了
    return;
  }
  // RGA Process 失败了，不掉头
  // 代码继续往下走，走 OpenCV 路径
}
// OpenCV 路径 ...
```

甚至 `Process()` 内部还有第二级降级：`imresize_t` 失败 → 尝试 `imcvtcolor`（只做色彩转换）。

这形成了一个三级降级链：

```
最优：RGA resize + BGR2RGB（imresize_t）
  ↓ 失败
次优：RGA 仅 BGR2RGB（imcvtcolor）
  ↓ 失败
兜底：OpenCV resize + cvtColor
```

---

## 9. 常见 RGA 问题排查

### 9.1 `/dev/rga` 权限问题

RGA 设备文件在 `/dev/rga`（有些板子是 `/dev/rga0`）。如果程序报 `RGA init failed`，先检查：

```bash
ls -la /dev/rga*
# 如果文件不存在 → 内核没加载 RGA 驱动
# 如果权限不够 → sudo chmod 666 /dev/rga
```

`c_RkRgaInit()` 内部会 `open("/dev/rga")`，如果文件不存在或权限不够，就会返回失败。

### 9.2 librga 库版本不匹配

Rockchip 的 RGA 库有多个版本，API 不完全兼容。常见差异：

| 版本 | 头文件 | 关键差异 |
|------|--------|---------|
| RGA 1.x | `rga/RgaApi.h` | `c_RkRgaInit()` 返回 0 表示成功 |
| RGA 2.x | `rga/RgaApi.h` | `c_RkRgaInit()` 返回 `IM_STATUS_SUCCESS` |
| RGA 3.x | `rga/im2d.hpp` | C++ API，函数签名不同 |

你的代码兼容了返回值的差异（`ret != IM_STATUS_SUCCESS && ret != 0`），但如果你拿到一块新板子，用的 RGA 库版本完全不同，可能还会出问题。

### 9.3 `imresize_t` 参数不一致

不同版本的 `imresize_t` 签名可能有差异。常见的坑：

- **scale 参数**：有些版本接受 float，有些接受 double
- **sync 参数**：有些版本叫 `sync`，有些叫 `block`
- **interpolation 参数**：枚举值名字可能不同（`INTER_LINEAR` vs `IM_INTERPOLATE_LINEAR`）

如果编译时报"参数不匹配"，检查你板子上 `rga/im2d.h` 的实际签名。

### 9.4 颜色不对 / 画面偏色

如果 RGA 输出画面颜色偏蓝或偏红，大概率是格式参数写错了：

- `RK_FORMAT_BGR_888` ←→ `RK_FORMAT_RGB_888`：搞反了 R 和 B 通道
- `RK_FORMAT_RGBA_8888` vs `RK_FORMAT_RGB_888`：4 字节还是 3 字节

你的代码里源是 `RK_FORMAT_BGR_888`（OpenCV 默认 BGR），目标是 `RK_FORMAT_RGB_888`，这是正确的。如果画面看起来偏蓝，检查是不是把源和目标格式写反了。

---

## 章节小结

RGA 是 Rockchip 平台上做图像预处理的"涡轮增压器"。核心要点：

1. **RGA 是 2D 硬件加速器**，能缩放、转格式、填充——不占 CPU
2. **三步生命周期**：`Init()` 初始化 → `Process()` 每帧调用 → `Release()` 清理
3. **`imresize_t` 一步完成**：缩放 + BGR→RGB 同时做，零额外拷贝
4. **快了大约 6 倍**，因为专用硬件 + DMA + 零拷贝
5. **三级自动降级**：RGA 全功能 → RGA 仅色彩转换 → OpenCV 兜底
6. **条件编译** `#ifdef RKFAST_USE_RGA` 保证在 PC 上也能编译
7. **降级不影响程序运行**：RGA 挂了就慢一点，程序照常工作

在你的项目里，RGA 和 OpenCV 降级路径共同保证了"最优性能和最大兼容性"的平衡——板子上跑 RGA 飞快，PC 上开发调试用 OpenCV，两边代码是同一套。
