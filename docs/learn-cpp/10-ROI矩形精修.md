# 10 — ROI 矩形精修：从粗粒度检测到像素级精确

## 问题出在哪

先回顾你的管线。RKNN 推理时，是把整帧 1920x1080 的图缩到 640x640 喂给 NPU。这个 resize 操作让目标在 NPU 眼里只有几十个像素。NPU 吐回来的边界框（bounding box），坐标精度大概是"啊，目标大概在这一带"的水平。

打个比方：你在航拍画面里，用高德地图搜"附近有哪些餐厅"——RKNN 告诉你"在东风路 100 号到 200 号之间"。这对于**找目标**够用了，但对于**打目标**不够——你需要精确到"正对第三个窗户"。

这就是 ROI 精修模块要干的事：拿到 RKNN 的粗检测结果，回到原始大图上，在检测框附近局部搜索，用传统图像处理的方法把目标的精确边界找出来。

本章对应文件：`rect_refiner.hpp` + `rect_refiner.cpp`。

---

## 整体思路：两条路径，一个评分

`RectRefiner` 类非常简单，对外只有一个 `Refine` 函数，输入一张图和一个"指导框"（guide_box），输出一个精确的四边形（或者输出空——表示没找到）。

内部是怎么干的？先放大指导框 24%（保证目标一定在框里），然后切出这个小区域，走两条路径：

- **路径 A（二值化路径）**：OTSU 自动阈值 → 形态学开闭运算 → 找轮廓
- **路径 B（边缘检测路径）**：Canny 边缘检测 → 膨胀连接断边 → 找轮廓

先走路径 A，如果找到了就结束；找不到再走路径 B。

为什么要两条路？因为目标的视觉特征不确定——有时是"浅色背景上的深色物体"（二值化效果好），有时是"复杂纹理上的边缘"（Canny 效果好）。两条路兜底，总有一条能抓住。

---

## Refine 函数逐步骤拆解

```cpp
std::optional<RefinedRect> RectRefiner::Refine(
    const cv::Mat& frame_bgr, const Box& guide_box,
    const std::optional<cv::Point2f>& reference_center) const {
```

参数说明：

| 参数 | 类型 | 含义 |
|------|------|------|
| `frame_bgr` | `const cv::Mat&` | 原始大图（只读引用，不复制） |
| `guide_box` | `const Box&` | RKNN 给的粗检测框 |
| `reference_center` | `std::optional<cv::Point2f>` | 可选的中心参考点，不传就用 guide_box 的中心 |

返回值：`std::optional<RefinedRect>` —— 里面要么装着精修结果，要么是空的（`std::nullopt`）。

### 步骤1：扩展 ROI 区域

```cpp
const float guide_w = BoxWidth(guide_box);
const float guide_h = BoxHeight(guide_box);
const float pad_x = guide_w * config_.roi_expand_ratio;   // 默认 0.24
const float pad_y = guide_h * config_.roi_expand_ratio;
const Box roi_box = ClipBox(
    Box{guide_box.x1 - pad_x, guide_box.y1 - pad_y,
        guide_box.x2 + pad_x, guide_box.y2 + pad_y},
    frame_bgr.cols, frame_bgr.rows);
```

这里做的事：把 RKNN 检测框向四周各扩大 24%。为什么是 24%？因为目标检测器的框通常偏紧，目标边缘可能刚好在框外面。扩大一点保证目标完整落在 ROI 里。

扩大之后马上调用 `ClipBox`（你在 types.hpp 里见过的），把越界的部分裁掉，保证 ROI 不超出画面。

### 步骤2：尺寸检查

```cpp
const int x = static_cast<int>(std::floor(roi_box.x1));
const int y = static_cast<int>(std::floor(roi_box.y1));
const int w = static_cast<int>(std::ceil(roi_box.x2) - std::floor(roi_box.x1));
const int h = static_cast<int>(std::ceil(roi_box.y2) - std::floor(roi_box.y1));
if (w < config_.roi_min_size || h < config_.roi_min_size) {
    return std::nullopt;
}

const cv::Rect roi(x, y, std::min(w, frame_bgr.cols - x),
                   std::min(h, frame_bgr.rows - y));
if (roi.width < config_.roi_min_size || roi.height < config_.roi_min_size) {
    return std::nullopt;
}
```

两次尺寸检查：

1. 第一次检查扩展后的宽高是否小于 `roi_min_size`（默认 24 像素）
2. 第二次在裁剪到画面边界后再检查一次

小于 24 像素的区域太小了，OpenCV 的形态学操作和轮廓查找都没法正常工作，直接放弃，返回空。

> **`static_cast<int>` 是什么？** C++ 有四种显式转换方式，`static_cast` 是最常用的"编译时类型转换"，相当于 C 的 `(int)x`，但更安全——编译器会帮你检查转换是否合法。

> **`std::floor` / `std::ceil` 是什么？** `<cmath>` 里的取整函数。`floor` 向下取整（朝负无穷方向），`ceil` 向上取整（朝正无穷方向）。这里用 floor 取 x1 确保左边界不会少切，用 ceil 取 x2 确保右边界不会少切。

### 步骤3：转为灰度并滤波

```cpp
const cv::Point2f target_center =
    reference_center.has_value() ? *reference_center : BoxCenter(guide_box);
const cv::Mat roi_bgr = frame_bgr(roi);

cv::Mat gray;
cv::cvtColor(roi_bgr, gray, cv::COLOR_BGR2GRAY);
cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0.0);
```

- `target_center`：目标的大致中心。如果外界传了参考中心就用外界的（跟踪器给的预测位置），否则用 guide_box 的中心
- `frame_bgr(roi)`：这是 `cv::Mat` 的 `operator()`，用矩形区域索引图像，得到一个 **不复制数据** 的视图（header 指向原图内存 + 不同的起始指针），非常高效
- `cvtColor`：BGR 转灰度，后续处理只需要亮度信息
- `GaussianBlur`：5x5 高斯模糊，消除噪声，让后续二值化和边缘检测更干净

### 步骤4：路径A — OTSU 二值化 + 形态学

```cpp
cv::Mat binary;
cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
cv::morphologyEx(binary, binary, cv::MORPH_OPEN,
                 cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
cv::morphologyEx(binary, binary, cv::MORPH_CLOSE,
                 cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

if (auto result = SearchContours(binary, roi, target_center)) {
    return result;
}
```

逐句翻译：

- **`cv::threshold(..., THRESH_BINARY_INV | THRESH_OTSU)`**：OTSU 是一种"自动选阈值"的二值化算法。它会分析图像的灰度直方分布，自己算出最合适的阈值，把图分成黑和白两类。加上 `_INV` 表示反转（黑的变白、白的变黑）——因为我们假设目标是深色的，反转后目标变成白色区域，方便轮廓查找。

- **`MORPH_OPEN`（开运算，3x3 核）**：先腐蚀再膨胀。效果是**去掉孤立的小白点（噪声）**。就像拿一把扫帚把桌上散落的小碎屑扫掉。

- **`MORPH_CLOSE`（闭运算，5x5 核）**：先膨胀再腐蚀。效果是**填补目标内部的小黑洞**。就像把裂开的饼干按回去，让目标区域完整。

- **`getStructuringElement(MORPH_RECT, Size(...))`**：形态学操作的"刷子"形状。这里用矩形核，因为目标（装甲板）本身就是矩形的。

- 最后调用 `SearchContours`，在二值图里找四边形。如果找到了（`auto result` 里有值），直接返回。

### 步骤5：路径B — Canny 边缘检测

```cpp
cv::Mat edge;
cv::Canny(gray, edge, 60.0, 160.0);
cv::dilate(edge, edge,
           cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
return SearchContours(edge, roi, target_center);
```

路径 A 没找到，就执行路径 B：

- **`cv::Canny(gray, edge, 60.0, 160.0)`**：Canny 边缘检测。两个阈值 60 和 160：低于 60 的梯度不是边缘，高于 160 的一定是边缘，介于两者之间的看是否和强边缘相连。这能提取出图像中物体边缘的线条。

- **`cv::dilate(edge, ..., 3x3)`**：Canny 输出的边缘线可能有断点，用膨胀把断开的边缘连起来，形成一个闭合的轮廓。

- 再次调用同一个 `SearchContours`，这次在边缘图上找四边形。

注意：路由 B 不检查结果，直接返回 `SearchContours` 的返回值——找到了就是找到了，找不到就是 `std::nullopt`。

### 为什么有两条路径

路径 A（OTSU 二值化）适合**目标与背景颜色差异明显**的场景。比如白色天空中的深色无人机——OTSU 能直接找到正确阈值，二值化后目标是一个完整的白色块。

路径 B（Canny 边缘）适合**目标纹理复杂但边缘清晰**的场景。比如迷彩装甲板上灯条，二值化可能把迷彩纹也当成目标区域，但 Canny 能抓住灯条的矩形边缘。

两条路互为备份，能大幅减少"看不清"的情况。

---

## SearchContours：在二值/边缘图里找最优四边形

这是 `RectRefiner` 的核心私有函数：

```cpp
std::optional<RefinedRect> RectRefiner::SearchContours(
    const cv::Mat& binary, const cv::Rect& roi,
    const cv::Point2f& target_center) const {
```

输入：一张二值图（要么是 OTSU 结果，要么是 Canny 结果）、ROI 的位置信息、目标参考中心。

### 步骤1：找轮廓

```cpp
std::vector<std::vector<cv::Point>> contours;
cv::findContours(binary, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
```

`findContours` 返回一个 `vector<vector<Point>>`——每个轮廓是一串点的数组。注意它的类型：外层 vector 是所有轮廓的列表，内层 vector 是每个轮廓的点序列。

`RETR_LIST` 表示"把所有轮廓都列出来，不管嵌套关系"。`CHAIN_APPROX_SIMPLE` 表示"用最少的点表示轮廓"（比如一条直线只用两个端点，省内存）。

### 步骤2：面积阈值过滤

```cpp
const float min_area = static_cast<float>(roi.area()) * config_.refine_min_area_ratio;

for (const auto& contour : contours) {
    const double contour_area = cv::contourArea(contour);
    if (contour_area < min_area) continue;
    const double perimeter = cv::arcLength(contour, true);
    if (perimeter < 20.0) continue;
```

- `roi.area()` 是整个 ROI 区域的面积
- `refine_min_area_ratio` 默认 0.05，即至少占 ROI 面积的 5%
- 周长小于 20 像素的轮廓也直接跳过——太小了，不可能是目标

### 步骤3：多边形逼近——找出四边形

```cpp
    std::vector<cv::Point> approx;
    cv::approxPolyDP(contour, approx, 0.02 * perimeter, true);
    if (approx.size() != 4 || !cv::isContourConvex(approx)) continue;
```

`approxPolyDP` 是 Douglas-Peucker 算法：用尽量少的点去逼近原始轮廓。参数 `0.02 * perimeter` 是逼近精度——允许近似多边形偏离原始轮廓的最大距离是周长的 2%。越接近 0，逼近越精确但点数越多。

然后两个条件：
1. `approx.size() != 4` —— 逼近结果不是 4 个点，不是四边形，扔掉
2. `!cv::isContourConvex(approx)` —— 不是凸多边形（有内凹），扔掉。因为我们找的是装甲板/灯条，一定是凸的

### 步骤4：最小尺寸检查

```cpp
    const cv::Rect rect = cv::boundingRect(approx);
    if (rect.width < config_.roi_min_size / 2 ||
        rect.height < config_.roi_min_size / 2) continue;
```

`boundingRect` 返回包住多边形的正外接矩形。再检查一次尺寸，这次阈值是 `12`（`roi_min_size / 2`）——比整体 ROI 检查更宽松，因为四边形包在 ROI 内部可以小一些。

### 步骤5：四维评分系统

这是 SearchContours 最核心的部分。通过四个维度给每个候选四边形打分，选出最优的那个：

```cpp
    const float extent =
        static_cast<float>(contour_area) /
        std::max(1.0f, static_cast<float>(rect.area()));
    const cv::Point2f center(roi.x + rect.x + rect.width * 0.5f,
                             roi.y + rect.y + rect.height * 0.5f);
    const float dist = static_cast<float>(cv::norm(center - target_center));
    const float dist_norm = dist / std::max(1.0f, static_cast<float>(roi.width + roi.height));
    const float center_score = std::max(0.0f, 1.0f - dist_norm);
    const float area_score =
        std::min(1.0f, static_cast<float>(contour_area) /
                           std::max(1.0f, static_cast<float>(roi.area()) * 0.35f));
    const float aspect = std::min(rect.width, rect.height) /
                         std::max(1.0f, static_cast<float>(std::max(rect.width, rect.height)));
    const float score = area_score * 0.45f + extent * 0.30f + center_score * 0.15f +
                        aspect * 0.10f;
```

四个评分维度逐个解释：

| 维度 | 变量 | 权重 | 含义 |
|------|------|------|------|
| 面积 | `area_score` | 45% | 轮廓面积越大越好，但不超过 ROI * 35% 就满分 |
| 矩形度 | `extent` | 30% | 轮廓面积 ÷ 外接矩形面积，越接近 1 越像矩形 |
| 中心距离 | `center_score` | 15% | 候选四边形中心离目标参考中心越近越好 |
| 宽高比 | `aspect` | 10% | 短边 ÷ 长边，越接近 1 越像正方形（装甲板灯条通常接近正方形） |

**逐项大白话解释：**

- **面积评分（0.45 权重，最重要）**：为什么最重要？因为我们找的是目标主体。面积太小可能是噪声碎片。`std::min(1.0f, area / (roi_area * 0.35))` 的意思是——面积达到 ROI 的 35% 就是满分，超大也不加分。因为你的目标在 ROI 里大概就占这么大。

- **矩形度评分（0.30 权重，第二重要）**：轮廓面积 ÷ 外接矩形面积。一个完美的矩形这个值接近 1.0（轮廓就是矩形的边，面积等于矩形面积）。一个弯曲的轮廓这个值小很多。这是判断"这东西是不是矩形"最直接的指标。

- **中心距离评分（0.15 权重）**：候选四边形中心离 RKNN 给的参考中心有多远。距离用 `宽度+高度` 归一化，保证和图像尺度无关。离得越近分越高——因为你知道目标大概就在参考中心附近。

- **宽高比评分（0.10 权重）**：min(宽,高) ÷ max(宽,高)。一个 1:1 的正方形这个值是 1.0。一个很扁的长条这个值就小。装甲板的灯条通常是近似正方形的，所以给接近 1:1 的形状加分。

每次遍历，如果当前候选的分数比之前最好的高，就记录下它。遍历结束后返回分数最高的那个。

### 步骤6：选出最优四边形

```cpp
    if (score <= best_score) continue;

    const auto corners = OrderCorners(approx);
    RefinedRect candidate;
    candidate.box = Box{static_cast<float>(roi.x + rect.x),
                        static_cast<float>(roi.y + rect.y),
                        static_cast<float>(roi.x + rect.x + rect.width),
                        static_cast<float>(roi.y + rect.y + rect.height)};
    candidate.center = center;
    candidate.corners = corners;
    candidate.score = score;
    best = candidate;
    best_score = score;
```

如果当前候选更好，就把它打包成 `RefinedRect`：
- `box`：外接矩形（注意坐标要加上 ROI 的偏移，因为矩形是相对于 ROI 的，要转回全图坐标）
- `center`：四边形的几何中心
- `corners`：四个角点（排好序的）
- `score`：综合评分

---

## OrderCorners：四角点排序

```cpp
std::array<cv::Point2f, 4> RectRefiner::OrderCorners(
    const std::vector<cv::Point>& polygon) {
  std::array<cv::Point2f, 4> corners{};
  std::array<cv::Point2f, 4> pts{};
  for (size_t i = 0; i < 4; ++i) {
    pts[i] = cv::Point2f(static_cast<float>(polygon[i].x),
                         static_cast<float>(polygon[i].y));
  }

  auto sum_cmp = [](const cv::Point2f& a, const cv::Point2f& b) {
    return (a.x + a.y) < (b.x + b.y);
  };
  auto diff_cmp = [](const cv::Point2f& a, const cv::Point2f& b) {
    return (a.y - a.x) < (b.y - b.x);
  };

  corners[0] = *std::min_element(pts.begin(), pts.end(), sum_cmp);   // 左上
  corners[2] = *std::max_element(pts.begin(), pts.end(), sum_cmp);   // 右下
  corners[1] = *std::min_element(pts.begin(), pts.end(), diff_cmp);  // 右上
  corners[3] = *std::max_element(pts.begin(), pts.end(), diff_cmp);  // 左下
  return corners;
}
```

这段代码的思想非常巧妙。在图像坐标系中（x 向右，y 向下）：

- 左上角（x 最小，y 最小）→ x + y 最小 → 用 `sum_cmp` 找最小值
- 右下角（x 最大，y 最大）→ x + y 最大 → 用 `sum_cmp` 找最大值
- 右上角（x 最大，y 最小）→ y - x 最小 → 用 `diff_cmp` 找最小值
- 左下角（x 最小，y 最大）→ y - x 最大 → 用 `diff_cmp` 找最大值

> **`auto sum_cmp = [](const cv::Point2f& a, const cv::Point2f& b) { ... };`** 这是 C++ 的 **lambda 表达式**——一个"就地定义的匿名函数"。它不在任何地方单独声明，直接写在 `[]{}` 里面，传给 `std::min_element` 当比较器。`[]` 是捕获列表（这里为空，不捕获外部变量），`()` 是参数列表，`{}` 是函数体。你项目里还有其他地方用到 lambda，后面遇到了再细讲。

`std::min_element` 和 `std::max_element` 是 `<algorithm>` 里的标准算法，给定一个范围和比较函数，返回最小/最大元素的迭代器。`*` 解引用拿到值。

排序后的结果：
```
corners[0] = 左上角
corners[1] = 右上角
corners[2] = 右下角
corners[3] = 左下角
```

---

## 为什么这块代码放在 CPU 上跑

你可能会问：RK3576 有 6TOPS 的 NPU，为什么不把这个也放 NPU 上？

答案：**ROI 区域太小了**。

整个图像可能有 1920x1080 = 200 万像素，但 ROI 区域通常只有 100x100 = 1 万像素。NPU 的优势是并行处理大量数据——数据量小的时候，数据在 CPU 和 NPU 之间搬来搬去的时间可能比直接算的时间还长。

而且 OpenCV 的 `threshold`、`morphologyEx`、`findContours` 这些函数在 CPU 上跑 1 万像素的区域，耗时不到 1 毫秒。上 NPU 反而画蛇添足。

这就是软硬件分工的基本逻辑：**大规模并行运算给 NPU，小规模像素级操作给 CPU。**

---

## OpenCV 函数速查

| 函数 | 用途 | 关键参数 | 在你代码里的位置 |
|------|------|----------|------------------|
| `cv::cvtColor(img, gray, COLOR_BGR2GRAY)` | 彩色转灰度 | 输入、输出、转换类型 | `rect_refiner.cpp:53` |
| `cv::GaussianBlur(src, dst, Size(5,5), 0)` | 高斯模糊降噪 | 输入、输出、核大小、sigma | `rect_refiner.cpp:54` |
| `cv::threshold(src, dst, 0, 255, OTSU\|BINARY_INV)` | OTSU 自动二值化 | 阈值自动选、最大值 255、模式 | `rect_refiner.cpp:57` |
| `cv::morphologyEx(src, dst, OPEN, kernel)` | 先腐蚀再膨胀（去噪） | 输入、输出、操作类型、核 | `rect_refiner.cpp:58` |
| `cv::morphologyEx(src, dst, CLOSE, kernel)` | 先膨胀再腐蚀（补洞） | 输入、输出、操作类型、核 | `rect_refiner.cpp:60` |
| `cv::Canny(src, dst, 60, 160)` | 边缘检测 | 输入、输出、低阈值、高阈值 | `rect_refiner.cpp:68` |
| `cv::dilate(src, dst, kernel)` | 膨胀（加粗/连接） | 输入、输出、核 | `rect_refiner.cpp:69` |
| `cv::findContours(binary, contours, RETR_LIST, SIMPLE)` | 找所有轮廓 | 二值图、输出、检索模式、压缩模式 | `rect_refiner.cpp:78` |
| `cv::contourArea(contour)` | 计算轮廓面积 | 轮廓（点向量） | `rect_refiner.cpp:86` |
| `cv::arcLength(contour, true)` | 计算轮廓周长 | 轮廓、是否闭合 | `rect_refiner.cpp:90` |
| `cv::approxPolyDP(contour, approx, eps, true)` | 多边形逼近 | 原始轮廓、输出、精度、是否闭合 | `rect_refiner.cpp:96` |
| `cv::isContourConvex(approx)` | 凸性检查 | 轮廓 | `rect_refiner.cpp:97` |
| `cv::boundingRect(approx)` | 正外接矩形 | 轮廓 | `rect_refiner.cpp:101` |
| `cv::norm(pt - center)` | 两点距离 | 向量差 | `rect_refiner.cpp:112` |

---

## 新增语法点：写在 class 里的 lambda

你在 `OrderCorners` 里看到了这种写法：

```cpp
auto sum_cmp = [](const cv::Point2f& a, const cv::Point2f& b) {
    return (a.x + a.y) < (b.x + b.y);
};
```

这是 C++ 的 **lambda 表达式**，本质上就是一个"没有名字的 inline 函数"。结构：

```
[捕获列表] (参数列表) -> 返回值类型 { 函数体 }
```

捕获列表 `[]`：lambda 体里能用哪些外部变量。空 `[]` 表示什么都不用，只用自己的参数。

对比传统写法：

```cpp
// 传统：在外面定义一个函数
bool sum_cmp_func(const cv::Point2f& a, const cv::Point2f& b) {
    return (a.x + a.y) < (b.x + b.y);
}
// 然后传函数名
std::min_element(pts.begin(), pts.end(), sum_cmp_func);

// Lambda：就地定义，更紧凑
auto sum_cmp = [](const cv::Point2f& a, const cv::Point2f& b) {
    return (a.x + a.y) < (b.x + b.y);
};
std::min_element(pts.begin(), pts.end(), sum_cmp);
```

两种写法效果一样。lambda 的优势是能写在逻辑旁边，不用跳来跳去找函数定义。

---

## 新增语法点：`std::optional` 配合 `if (auto result = ...)` 的简写

```cpp
if (auto result = SearchContours(binary, roi, target_center)) {
    return result;
}
```

这是 C++ 的一个便利写法，等价于：

```cpp
auto result = SearchContours(binary, roi, target_center);
if (result.has_value()) {     // std::optional 在 if 里自动转成 bool
    return *result;            // 解引用取出里面的值
}
```

`if` 条件里的变量声明是 C++17 的"if with initializer"——先在 `if` 里声明变量，然后自动用该变量做条件判断。`std::optional` 重载了 `operator bool`，有值时返回 `true`，为空时返回 `false`。

`return result;` 这里没有写 `*result`（解引用），因为编译器看到返回类型也是 `std::optional<RefinedRect>`，自动帮你把值打包回 optional。

---

## 小结

这章你学到了一个完整的传统图像处理精修流程：

1. **ROI 扩展** → 把检测框放大 24%，保证目标在区域内
2. **裁剪与检查** → `ClipBox` 约束、`roi_min_size` 兜底
3. **双路径策略** → OTSU 二值化优先，Canny 边缘兜底
4. **轮廓筛选** → 面积阈值 → 多边形逼近 → 四边形 + 凸性检查
5. **四维评分** → 面积(45%) + 矩形度(30%) + 中心距离(15%) + 宽高比(10%)
6. **四角排序** → sum_cmp + diff_cmp 标准套路

整个 `RectRefiner` 只有 160 多行代码，但每一步都有明确的数学模型和实际意义。这就是"算法工程化"的本质——不是套一个复杂的深度学习模型，而是用最合适的经典算法，组合成一个可靠的轻量级解决方案。
