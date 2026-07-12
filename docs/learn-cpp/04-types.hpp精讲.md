# 04 -- types.hpp 精讲：项目的"共同语言"

## types.hpp 在项目里的位置

打开项目目录看一眼，`include/rkfast/types.hpp` 这个文件被几乎所有模块 `#include`：

```
camera_source.hpp  ──┐
rknn_detector.hpp  ──┼── #include "rkfast/types.hpp"
tracker.hpp        ──┤
rect_refiner.hpp   ──┘
```

它是全项目的**共同语言** -- 所有模块之间传递数据用的结构体、所有公共的数学小工具，都定义在这里。没有它，CameraSource 拍到的东西 Detector 不认识，Detector 检测出的结果 Tracker 看不懂。

你可以把它想象成一份合同：Box 是什么、Detection 长什么样、IOU 怎么算 -- 大家签字画押，按统一标准办事。

文件后缀 `.hpp` 是 C++ 头文件的习惯写法（等价于 `.h`），表示"这里面全是 C++ 的东西（类、模板、namespace）"。

---

## Box 结构体：为什么不写成 class

```cpp
struct Box {
  float x1 = 0.0f;
  float y1 = 0.0f;
  float x2 = 0.0f;
  float y2 = 0.0f;
};
```

首先看到的是 C++ 的成员默认值：`= 0.0f` 表示如果创建一个 Box 不赋值，坐标默认全是 0。这在 C 语言里做不到。

**为什么是 `struct` 而不是 `class`？** 因为 Box 是一个纯数据容器，没有需要隐藏的内部状态。struct 默认全公开（public），正好符合"数据人人能读能写"的需求。C++ 社区的习惯是：纯数据类型用 struct，有复杂行为和状态的用 class。

四个字段的含义：

```
  (x1, y1) ───────────┐
    │                  │
    │       Box        │   height = y2 - y1
    │                  │
    └─────────── (x2, y2)
         width = x2 - x1
```

- **x1, y1**：左上角坐标（像素），原点在图像左上角
- **x2, y2**：右下角坐标（像素），注意是右下角，不是宽高

为什么存坐标而不是 `(x, y, w, h)`？两种表示法各有优劣。`(x1,y1,x2,y2)` 算 IOU 更方便，`(x,y,w,h)` 画框更方便。项目选择了前者，因为 IOU 是每次 NMS 都要算的高频操作。

---

## Detection 结构体：一次检测结果的完整档案

```cpp
struct Detection {
  Box box;
  cv::Point2f center{0.0f, 0.0f};
  float conf = 0.0f;
  int class_id = 0;
  int candidate_count = 0;
};
```

- **box**：检测框，类型就是上面刚讲的 Box。这是最核心的信息 -- 目标在哪里、有多大。
- **center**：框的中心点坐标。注意类型是 `cv::Point2f`（OpenCV 的浮点坐标），不是 Box 的四角。后续跟踪器用中心点做位置预测，所以提前算好存起来，不用每次都 `(x1+x2)/2`。
- **conf**：置信度（confidence），范围 0.0~1.0。神经网络输出的"我有多大把握这是一个目标"的概率。项目中选用 0.25 作为筛选门槛。
- **class_id**：类别 ID。你的模型只检测一种目标（装甲板灯条），所以这个值要么是 0（检测到了），要么不应该出现在结果列表里。如果模型支持多类，这个字段就是区分类别的关键。
- **candidate_count**：候选框数量。NMS（非极大值抑制）之前，可能同一个目标被多个锚框命中。这个字段告诉你"这个检测框合并了多少个候选"-- 数字越大说明神经网络对这个位置越"激动"。

---

## RefinedRect：Detection 的"精修版"

```cpp
struct RefinedRect {
  Box box;
  cv::Point2f center{0.0f, 0.0f};
  std::array<cv::Point2f, 4> corners{};
  float score = 0.0f;
};
```

RefinedRect 比 Detection 多了两个关键能力：

**corners -- 四角点**：类型是 `std::array<cv::Point2f, 4>`，即"4 个浮点坐标的数组"。神经网络输出的 Detection 是**轴对齐矩形**（边和图像边框平行），但实际目标可能是旋转的。RectRefiner 模块对矩形区域做传统图像处理（轮廓搜索），输出**旋转的四角点**。这 4 个点按固定顺序排列（左上、右上、右下、左下），下游模块可以直接用来画四边形或做透视变换。

**score -- 精修得分**：区别于 Detection 的 conf（神经网络置信度），score 是传统图像处理算法给出的评分（比如边缘清晰度、轮廓完整性）。两者互补：conf 靠深度特征，score 靠几何特征。

注意 RefinedRect 没有 class_id 和 candidate_count -- 它的上游已经是单个 Detection 了，不需要这些信息。

---

## TrackSnapshot：跟踪器的对外快照

```cpp
struct TrackSnapshot {
  bool active = false;
  Box box;
  cv::Point2f center{0.0f, 0.0f};
  float conf = 0.0f;
  int count = 0;
  int misses = 0;
  std::string state = "SEARCHING";
  std::string source = "none";
};
```

这个是 `RectTracker::Snapshot()` 的返回值，是跟踪器向外部汇报的"当前跟踪状态报告"。

**state 字段**是本项目的核心语义，它有三种取值：

| state | 含义 | 触发条件 |
|-------|------|----------|
| `"SEARCHING"` | 搜索中，还没锁定目标 | 跟踪器刚创建，或丢失目标后重新搜索 |
| `"LOCKED"` | 已锁定，稳定跟踪中 | 连续多帧检测到目标，跟踪器认为"稳了" |
| `"LOST"` | 目标丢失 | 连续 N 帧没检测到目标（但跟踪器还在预测位置） |

这就是一个简单的**状态机**：刚启动是 SEARCHING，收到稳定检测后切到 LOCKED，连续没检测到就变 LOST。LOST 状态下跟踪器仍然用 α-β 滤波预测目标位置，所以框不会立即消失，而是"惯性滑行"一段时间。

其他字段：
- **active**：跟踪器是否激活（false 表示根本没在工作）
- **count**：LOCKED 后累计的有效检测帧数（越大说明跟踪越稳定）
- **misses**：连续未检测到的帧数（越大说明越危险，快到 LOST 了）
- **source**：兜底信息来源，可以是 `"rknn"`（RKNN 直接检测结果）或 `"predict"`（滤波预测结果）或 `"none"`

---

## InferTimings 和 InferResult：性能和结果

```cpp
struct InferTimings {
  double preprocess_ms = 0.0;
  double rknn_ms = 0.0;
  double postprocess_ms = 0.0;
  double select_ms = 0.0;
  double total_ms = 0.0;
};

struct InferResult {
  uint64_t seq = 0;
  std::optional<Detection> detection;
  double infer_fps = 0.0;
  double timestamp = 0.0;
  InferTimings timings;
};
```

**InferTimings** 记录一次推理的各环节耗时（单位：毫秒），这是**性能监控**的核心数据：

```
相机采集 → 预处理 → RKNN推理 → 后处理 → 选择最优 → 跟踪 → 输出
           ↑preprocess_ms ↑rknn_ms ↑postprocess_ms ↑select_ms
           └─────────────────total_ms──────────────────┘
```

- **preprocess_ms**：预处理耗时（RGA 硬件 resize/颜色转换）
- **rknn_ms**：RKNN NPU 推理耗时（核心耗时）
- **postprocess_ms**：后处理耗时（解码输出、NMS）
- **select_ms**：从候选框中选择最优检测的耗时
- **total_ms**：总耗时（求和）

**InferResult** 是一次推理的完整输出包：

- **seq**：序列号（uint64_t），每帧递增，用于追踪丢帧
- **detection**：`std::optional<Detection>` -- 注意是 optional！这帧可能没检测到任何目标，就返回空（std::nullopt）。这个设计贯穿全项目：不确定有没有结果时，用 optional 比用 NULL 指针语义清晰得多。
- **infer_fps**：推理频率（帧/秒），用平滑算法计算避免剧烈抖动
- **timestamp**：推理完成的时间戳（秒），用高精度时钟记录
- **timings**：嵌套的 InferTimings，提供详细的分阶段耗时

---

## NowSeconds() -- 高精度时间戳

```cpp
inline double NowSeconds() {
  using Clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}
```

逐层拆解：

1. `std::chrono::steady_clock` -- C++11 引入的时间库里的**单调时钟**。它保证时间只会往前走，不会因为系统校时（NTP）而倒退。对应 C 语言的 `clock_gettime(CLOCK_MONOTONIC, ...)`。
2. `Clock::now()` -- 获取当前时间点，返回一个 `time_point` 对象。
3. `.time_since_epoch()` -- 计算这个时间点到"纪元"（通常是系统启动时刻）的差值，返回一个 `duration` 对象。
4. `std::chrono::duration<double>` -- 把这个差值转换成**以秒为单位的 double**。`duration<double>` 的意思是"单位是秒，数值类型是 double"。
5. `.count()` -- 从 duration 对象里把数值取出来，得到 `double` 类型的秒数。

返回值的精度：steady_clock 在 Linux 上通常是纳秒级分辨率，转成 double 秒可以精确到小数点后 9 位，足以测量微秒级的耗时。

项目里用它来记录每帧的时间戳和计算 FPS。

---

## ClampFloat() -- 把数值关在笼子里

```cpp
inline float ClampFloat(float value, float low, float high) {
  return std::max(low, std::min(high, value));
}
```

这是一个嵌套调用，从内往外读：

1. `std::min(high, value)` -- 取 value 和 high 中较小的那个。如果 value 超过 high，就返回 high。
2. `std::max(low, ...)` -- 把上一步的结果和 low 比较，取较大的。如果 value 小于 low，返回 low。

整体效果：**把 value 限制在 [low, high] 区间内**。

```
如果 value < low   →  返回 low
如果 value > high  →  返回 high
否则               →  返回 value
```

这个方法保证了后续 ClipBox 的计算结果永远在画面边界内。

---

## BoxWidth / BoxHeight / BoxArea -- 基础几何

```cpp
inline float BoxWidth(const Box& box)  { return std::max(1.0f, box.x2 - box.x1); }
inline float BoxHeight(const Box& box) { return std::max(1.0f, box.y2 - box.y1); }
inline float BoxArea(const Box& box)   { return BoxWidth(box) * BoxHeight(box); }
```

三个函数都接收 `const Box&` -- 只读引用，不复制 Box。

**为什么 `std::max(1.0f, ...)`？** 防止宽或高为负数或零。如果 x2 < x1（框的坐标倒挂了），`x2 - x1` 是负数，面积就乱套了。用 `max(1.0f, ...)` 保证宽和高至少是 1 像素。虽然返回值是浮点数，但 1 像素保证了面积至少是正数，避免后面 IOU 计算里出现除零或负数面积。

---

## BoxCenter() -- 返回中心点

```cpp
inline cv::Point2f BoxCenter(const Box& box) {
  return cv::Point2f((box.x1 + box.x2) * 0.5f, (box.y1 + box.y2) * 0.5f);
}
```

公式非常直白：中心点 X = (左 + 右) / 2，Y = (上 + 下) / 2。

返回类型 `cv::Point2f` 是 OpenCV 定义的二维浮点坐标，相当于一个 `{float x, float y}` 结构体。

注意项目中 Detection 和 RefinedRect 都存了 center 字段，但 Box 本身没有 center -- 需要用这个函数当场算。设计考虑：Box 只存四角坐标（最小存储），用到中心点时再算（O(1) 开销可以忽略）。

---

## ClipBox() -- 把越界的框拉回画面

```cpp
inline Box ClipBox(const Box& box, int frame_w, int frame_h) {
  Box clipped;
  clipped.x1 = ClampFloat(box.x1, 0.0f, static_cast<float>(std::max(0, frame_w - 1)));
  clipped.y1 = ClampFloat(box.y1, 0.0f, static_cast<float>(std::max(0, frame_h - 1)));
  clipped.x2 = ClampFloat(box.x2, 0.0f, static_cast<float>(std::max(0, frame_w - 1)));
  clipped.y2 = ClampFloat(box.y2, 0.0f, static_cast<float>(std::max(0, frame_h - 1)));
  if (clipped.x2 < clipped.x1) {
    std::swap(clipped.x1, clipped.x2);
  }
  if (clipped.y2 < clipped.y1) {
    std::swap(clipped.y1, clipped.y2);
  }
  return clipped;
}
```

**为什么需要裁剪？** 神经网络输出的检测框坐标可能超出画面边界。比如目标在画面边缘时，框的一半在画面外，坐标出现负数或大于画面宽高。如果直接把这种框交给后续处理（画框、裁剪 ROI），OpenCV 会报错或画出奇怪的结果。

逐段解析：

**第一步：四角裁剪**。把 x1 限制在 `[0, frame_w-1]`，如果小于 0 就改成 0，大于 frame_w-1 就改成 frame_w-1。为什么是 `frame_w - 1`？因为像素坐标从 0 开始，宽度 640 的画面，最右边的像素坐标是 639。

`static_cast<float>()` 是 C++ 风格的类型转换。`std::max(0, frame_w - 1)` 防止 frame_w 为 0 时出现负数边界。

**第二步：交换矫正**。裁剪后可能出现 x2 < x1 的情况（比如框原本完全在画面左侧，x2 被裁剪到 0，但 x1 也是 0，正常；但如果 x1 被裁到更大的值超过了 x2，就会出现倒挂）。`std::swap` 直接交换两个值，保证 x1 <= x2，y1 <= y2。

---

## BoxIou() -- 重点：IOU 算法

```cpp
inline float BoxIou(const Box& a, const Box& b) {
  const float ix1 = std::max(a.x1, b.x1);
  const float iy1 = std::max(a.y1, b.y1);
  const float ix2 = std::min(a.x2, b.x2);
  const float iy2 = std::min(a.y2, b.y2);
  const float iw = std::max(0.0f, ix2 - ix1);
  const float ih = std::max(0.0f, iy2 - iy1);
  const float inter = iw * ih;
  if (inter <= 0.0f) {
    return 0.0f;
  }
  const float uni = BoxArea(a) + BoxArea(b) - inter;
  return inter / std::max(uni, 1e-6f);
}
```

### IOU 是什么

IOU = **Intersection Over Union** = 交集面积 / 并集面积。

两个框重叠得越多，IOU 越接近 1。完全不重叠，IOU 为 0。

```
     框A                    框B                    交集          并集
  ┌────────┐            ┌────────┐            ┌────────┐    ┌──────────────┐
  │        │            │        │            │    ┌───│────│──┐           │
  │   A    │     +      │   B    │     =      │    │交 │    │  │    并     │
  │        │            │        │            │    └───│────│──┘           │
  └────────┘            └────────┘            └────────┘    └──────────────┘

  IOU = 交集 / 并集
```

### 为什么 NMS 需要它

NMS（Non-Maximum Suppression，非极大值抑制）的工作原理：

1. 神经网络输出了 N 个检测框，很多框是"同一个目标被反复检测"（重叠度很高）
2. 按置信度从高到低排序
3. 取置信度最高的框 A，然后遍历剩下的框：如果某个框 B 和 A 的 IOU > 阈值（比如 0.5），说明 B 是和 A 检测的同一个目标，把 B 扔掉
4. 重复，直到所有框处理完

所以 IOU 是 NMS 的核心判断依据 -- 两个框是不是"同一个目标"。

### 逐行解析

**第 1-4 行：计算交集矩形的四角**

```cpp
const float ix1 = std::max(a.x1, b.x1);  // 交集左上角 X：取两个框左边界中靠右的那个
const float iy1 = std::max(a.y1, b.y1);  // 交集左上角 Y：取两个框上边界中靠下的那个
const float ix2 = std::min(a.x2, b.x2);  // 交集右下角 X：取两个框右边界中靠左的那个
const float iy2 = std::min(a.y2, b.y2);  // 交集右下角 Y：取两个框下边界中靠上的那个
```

画图理解：

```
  a.x1        b.x1              a.x2        b.x2
    │           │                 │           │
    v           v                 v           v
    └─ A ───────│────────┐        └─────┬─────│── B ─┘
    │           └── B ───│──┐     │     └── A ─│──────┘
    │           │  交集   │  │     │     交集   │
    │           └─────────┘  │     │           │
    └────────────────────────┘     └───────────┘
    
    ix1 = max(a.x1, b.x1)          ix2 = min(a.x2, b.x2)
    取靠右的那个                    取靠左的那个
```

**第 5-6 行：交集宽高，负值清零**

```cpp
const float iw = std::max(0.0f, ix2 - ix1);  // 交集宽度，最小为 0
const float ih = std::max(0.0f, iy2 - iy1);  // 交集高度，最小为 0
```

如果两个框完全不重叠，ix2 < ix1（交集的两个角倒挂了），`ix2 - ix1` 是负数。`max(0.0f, ...)` 把它清零，最终 inter = 0。

**第 7 行：交集面积**

```cpp
const float inter = iw * ih;
```

**第 8-10 行：零交集短路**

```cpp
if (inter <= 0.0f) {
  return 0.0f;
}
```

提前返回，省去后续计算。

**第 11-12 行：并集面积 + 最终除法**

```cpp
const float uni = BoxArea(a) + BoxArea(b) - inter;
return inter / std::max(uni, 1e-6f);
```

并集 = A面积 + B面积 - 交集（因为交集被加了两次，要减掉一次）。

`std::max(uni, 1e-6f)` 防止除以零（两个框面积都是 0 的异常情况）。`1e-6f` = 0.000001，一个极小正数。

---

## SmoothValue() -- 指数移动平均

```cpp
inline double SmoothValue(double previous, double current, double alpha) {
  return previous <= 0.0 ? current : previous * (1.0 - alpha) + current * alpha;
}
```

这是**指数移动平均（EMA）**的一行实现。公式：

```
smooth = previous * (1-α) + current * α
```

其中 α 是平滑系数（0~1）：
- α = 0：完全不动，新值不影响
- α = 1：完全跟新值走，不保留历史
- α = 0.2（项目中 FPS 的默认值）：历史占 80%，新值占 20%

项目里用它平滑 FPS 显示：

```cpp
infer_fps_ = SmoothValue(infer_fps_, instant_fps, 0.2);
```

如果每帧直接显示瞬时 FPS（1000ms / 这帧耗时），数值会疯狂跳动（120→250→80→180...）。用 EMA 平滑后，显示的是一个"惯性"的值，人眼看着舒服。

**哨兵值处理**：`previous <= 0.0` 表示"这是第一帧"。此时没有历史数据，直接返回当前值，不套公式。

---

## 速查表

| 名称 | 类型 | 一句话解释 |
|------|------|-----------|
| `Box` | struct | 四角坐标 (x1,y1,x2,y2)，目标框的最小表示 |
| `Detection` | struct | Box + 中心点 + 置信度 + 类别 + 候选数 |
| `RefinedRect` | struct | Detection 精修版，多了四角点和传统算法评分 |
| `TrackSnapshot` | struct | 跟踪器状态报告，关键是 state 字段 |
| `InferTimings` | struct | 各阶段耗时（毫秒），性能调试用 |
| `InferResult` | struct | 一次推理的完整输出，含 optional<Detection> 和耗时 |
| `NowSeconds()` | inline 函数 | 高精度时间戳（秒），steady_clock |
| `ClampFloat(v,l,h)` | inline 函数 | 把 v 限制在 [l, h] 区间 |
| `BoxWidth(b)` | inline 函数 | 框宽，至少 1 |
| `BoxHeight(b)` | inline 函数 | 框高，至少 1 |
| `BoxArea(b)` | inline 函数 | 框面积 |
| `BoxCenter(b)` | inline 函数 | 框中心点 (cv::Point2f) |
| `ClipBox(b,w,h)` | inline 函数 | 裁剪到画面内，修正倒挂坐标 |
| `BoxIou(a,b)` | inline 函数 | IOU = 交集/并集，NMS 核心 |
| `SmoothValue(prev,cur,α)` | inline 函数 | EMA 平滑，FPS 显示用 |

---

## 小结

types.hpp 只有 117 行，代码很少但被全项目依赖。它干了三件事：

1. **定义数据格式**：Box、Detection、RefinedRect、TrackSnapshot、InferTimings、InferResult -- 六种结构体组成了项目的数据流骨架。
2. **提供几何工具**：宽高面积中心裁剪 IOU -- 全部 inline，频繁调用零开销。
3. **提供通用工具**：时间戳、数值钳位、指数平滑 -- 短小精悍的函数，哪里都能用。

读懂这一篇，后面所有模块的输入输出你就都认识了。Detection 从 Detector 流出，经过 Tracker 变成 TrackSnapshot，经过 RectRefiner 变成 RefinedRect -- 数据流的核心节点都在这一页。

---

## 练习 B：手写 IOU 计算器

新建 `practice_b_iou.cpp`，内容如下：

```cpp
#include <iostream>
#include <algorithm>  // std::max, std::min

struct Box {
    float x1 = 0.0f, y1 = 0.0f, x2 = 0.0f, y2 = 0.0f;

    float width() const { return std::max(1.0f, x2 - x1); }
    float height() const { return std::max(1.0f, y2 - y1); }
    float area() const { return width() * height(); }
};

// ===== 你要实现的核心函数 =====
float box_iou(const Box& a, const Box& b) {
    // 第1步：计算交集矩形的四角
    float ix1 = std::max(a.x1, b.x1);
    float iy1 = std::max(a.y1, b.y1);
    float ix2 = std::min(a.x2, b.x2);
    float iy2 = std::min(a.y2, b.y2);

    // 第2步：交集宽高
    float iw = std::max(0.0f, ix2 - ix1);
    float ih = std::max(0.0f, iy2 - iy1);

    // 第3步：交集面积
    float inter = iw * ih;
    if (inter <= 0.0f) return 0.0f;

    // 第4步：并集面积 = A + B - 交集
    float uni = a.area() + b.area() - inter;

    // 第5步：IOU = 交集 / 并集
    return inter / std::max(uni, 1e-6f);
}

int main() {
    // 测试1：两个框重叠一半
    Box a{0, 0, 100, 100};    // 100x100
    Box b{50, 50, 150, 150};  // 100x100
    // 交集：50x50 = 2500，并集：10000+10000-2500=17500
    // IOU = 2500/17500 ≈ 0.1429
    std::cout << "IOU(重叠一半): " << box_iou(a, b) << "\n";

    // 测试2：完全不重叠
    Box c{0, 0, 10, 10};
    Box d{100, 100, 110, 110};
    std::cout << "IOU(不重叠):   " << box_iou(c, d) << "\n";  // 预期 0

    // 测试3：完全重叠
    Box e{10, 10, 50, 50};
    Box f{10, 10, 50, 50};
    std::cout << "IOU(完全重叠): " << box_iou(e, f) << "\n";  // 预期 1.0

    return 0;
}
```

编译运行：

```bash
g++ -std=c++17 -o practice_b practice_b_iou.cpp && ./practice_b
```

预期输出：

```
IOU(重叠一半): 0.142857
IOU(不重叠):   0
IOU(完全重叠): 1
```
