# 08 -- RKNN 推理全流程：从模型文件到检测结果

## 1. RKNN 是什么

你平时写的 C 程序，是在 CPU 上跑的。CPU 是"通用计算器"——什么都能算，但专项任务不够快。

AI 模型推理（让神经网络"看图找东西"）需要海量的矩阵乘法和卷积运算。这种活儿，CPU 干起来很吃力。于是芯片厂商在 SoC（系统级芯片）里塞了一个专门的硬件模块——**NPU（神经网络处理器）**，专门加速 AI 推理。

但问题是：你从网上下载的 AI 模型（比如 YOLO 的 `.pt` 文件），NPU 不认识。NPU 有自己的"方言"。

**RKNN 就是这个翻译过程**：

```
你训练好的模型                     NPU 能跑的文件
(PyTorch/TensorFlow/.pt)  ──→  (.rknn)
                         RKNN Toolkit
                         编译/量化
```

Rockchip 提供了两样东西：
- **RKNN Toolkit**（PC 上跑）：把你的 PyTorch 模型转换成 `.rknn` 文件，放在电脑上跑
- **RKNN API**（板子上跑）：也就是 `librknnrt.so` 这个库，在 3576 板子上加载 `.rknn` 文件，驱动 NPU 干活

你项目里用的就是后半段——在板子上调 RKNN API 做推理。模型文件已经提前转好了，放在 `models/best_rk3576_int8.rknn`。

---

## 2. 整体管线：6 步走完一次推理

你的 `RknnDetector` 类封装了一整套流程。一句话概括：

> 把摄像头画面喂给 NPU，NPU 跑模型，输出"画面里有没有目标，在哪儿"。

整个流程可以画成一条流水线：

```
加载模型 → 初始化NPU → 设置核心 → 预处理 → 推理 → 解码+NMS+选择
LoadModel   Open()     core_mask  Preprocess Infer  Decode+Choose
  (1)        (2)         (3)        (4)       (5)       (6)
```

- **(1) LoadModel**：把 `.rknn` 文件从磁盘读进内存
- **(2) rknn_init**：初始化 NPU 上下文。相当于"开机启动 NPU，把模型装进去"
- **(3) rknn_set_core_mask**：告诉 NPU 用几个核心干活
- **(4) Preprocess**：把摄像头画面缩放到模型输入尺寸，做 letterbox、色彩转换
- **(5) rknn_inputs_set → rknn_run → rknn_outputs_get**：把数据喂进去、跑推理、拿结果
- **(6) DecodeOutputs + ChooseDetection**：把 NPU 输出的"原始数字"翻译成"目标框坐标"

步骤 (1)-(3) 只在程序启动时执行一次，步骤 (4)-(6) 每帧都执行。

---

## 3. LoadModel：把 .rknn 文件读进内存

```cpp
bool RknnDetector::LoadModel(std::string* error) {
  std::ifstream ifs(config_.model_path, std::ios::binary);  // ①
  if (!ifs) {
    if (error != nullptr) {
      *error = "could not open model: " + config_.model_path;
    }
    return false;
  }
  model_data_ = std::vector<uint8_t>(std::istreambuf_iterator<char>(ifs),  // ②
                                     std::istreambuf_iterator<char>());
  if (model_data_.empty()) {
    if (error != nullptr) {
      *error = "model file is empty: " + config_.model_path;
    }
    return false;
  }
  return true;
}
```

### ① 为什么必须用 `std::ios::binary`？

`.rknn` 文件是二进制格式，里面装的不是文字，而是编译后的 NPU 指令和量化参数。

如果你不用 `binary` 模式（也就是文本模式），Windows 上读取时遇到字节 `0x1A` 会被当成"文件结束符"，数据就读不全了。Linux 上虽然文本和二进制模式行为一样，但为了代码跨平台可移植，显式写 `binary` 是好习惯。

一句话：**模型文件是二进制数据，不是文本，用 binary 模式打开。**

### ② 一行代码把整个文件读进 `vector<uint8_t>`

```cpp
model_data_ = std::vector<uint8_t>(std::istreambuf_iterator<char>(ifs),
                                   std::istreambuf_iterator<char>());
```

这是一个经典的 C++ 技巧。解释一下：

- `std::istreambuf_iterator<char>(ifs)` 指向文件流的"第一个字节"
- `std::istreambuf_iterator<char>()`（不带参数）是一个特殊的"哨兵"，表示"流的末尾"
- 把这两个迭代器传给 `vector` 的构造函数，vector 就会从第一个字节开始，一直读到哨兵，把全部数据拷进去

等价于你用 C 语言写的 `fread` + `malloc`，但更安全——vector 自动管理内存，函数返回后也不会泄漏。

最后数据存在了 `std::vector<uint8_t> model_data_` 这个成员变量里。`uint8_t` 就是无符号单字节，也就是"原始字节"——跟 `unsigned char` 是一回事。

---

## 4. Open 函数精讲：初始化 NPU

`Open()` 是整个类里最"重"的函数，做的事多，但逻辑清晰：

```cpp
bool RknnDetector::Open(std::string* error) {
  if (!LoadModel(error)) {           // ① 加载模型文件
    return false;
  }
  int ret = rknn_init(&context_, model_data_.data(),  // ② 初始化NPU
                      static_cast<uint32_t>(model_data_.size()), 0, nullptr);
  // ... 错误处理 ...
  // ③ 设置NPU核心
  // ④ QueryTensors 查询输入输出形状
  // ⑤ 分配缓冲区
  // ⑥ 尝试RGA硬件初始化
  return true;
}
```

### ① LoadModel —— 上一步

### ② `rknn_init()` —— NPU 的"开机"

```cpp
int ret = rknn_init(&context_,           // 输出：NPU上下文句柄
                    model_data_.data(),  // 输入：模型数据指针
                    model_data_.size(),  // 输入：模型数据大小
                    0,                   // 标志位（不用自定义标志就传0）
                    nullptr);            // 扩展参数（不用就传nullptr）
```

这一行的作用：把刚才读进来的 `.rknn` 模型"加载到 NPU 上"。成功后，`context_` 里存了一个句柄（handle），后面所有 NPU 操作都要用它。你可以把 `context_` 理解成一个"NPU 会话 ID"——有这个 ID，才能跟 NPU 对话。

失败就返回错误码，你项目里用 `RKNN_SUCC`（=0）检查是否成功。

### ③ `rknn_set_core_mask()` —— 让几个 NPU 核心干活

RK3576 有多个 NPU 核心。你可以指定用几个：

```cpp
// ResolveCoreMask 函数在 rknn_detector.cpp:33-63
if (name == "0")   return RKNN_NPU_CORE_0;       // 只用核心0
if (name == "1")   return RKNN_NPU_CORE_1;       // 只用核心1
if (name == "01")  return RKNN_NPU_CORE_0_1;     // 核心0+1 双核
if (name == "012") return RKNN_NPU_CORE_0_1_2;   // 核心0+1+2 三核
if (name == "all") return RKNN_NPU_CORE_ALL;     // 所有核心
if (name == "auto") return RKNN_NPU_CORE_AUTO;   // 自动选择
```

你的配置里默认是 `"default"`，也就是不调 `rknn_set_core_mask`，让 NPU 自己决定。

`#ifdef RKFAST_RKNN_HAS_CORE_MASK` 是用来做条件编译的——因为旧版 RKNN API 可能没有这个函数。编译时如果定义了宏，就编译这段代码；否则跳过，打印"API not available"。

### ④ QueryTensors —— 问 NPU："输入输出各长什么样？"

模型装进 NPU 后，你需要的下一件事是：**搞清楚模型的输入输出格式**。

```cpp
bool RknnDetector::QueryTensors(std::string* error) {
  // 第一步：问"有几个输入，几个输出"
  int ret = rknn_query(context_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
  if (io_num_.n_input != 1) {  // 你的模型只有一个输入（一张图）
    // 报错
  }

  // 第二步：问"输入张量的详细信息"
  input_attr_.index = 0;
  ret = rknn_query(context_, RKNN_QUERY_INPUT_ATTR, &input_attr_, sizeof(input_attr_));
  // 从 input_attr_ 里解析出：
  //   - fmt: 数据排列方式是 NCHW 还是 NHWC
  //   - dims: 各维度大小 —— [1, 3, 640, 640] 或 [1, 640, 640, 3]
  //   - type: 数据类型，UINT8/INT8/FLOAT16/FLOAT32

  // 第三步：问"每个输出张量的详细信息"
  for (uint32_t i = 0; i < io_num_.n_output; ++i) {
    output_attrs_[i].index = i;
    ret = rknn_query(context_, RKNN_QUERY_OUTPUT_ATTR, &output_attrs_[i], ...);
  }
}
```

**你项目里模型的输入**是 `[1, 3, 640, 640]`（NCHW 格式），含义：

| 维度 | 值 | 含义 |
|------|----|------|
| N（批次） | 1 | 一次推理一张图 |
| C（通道） | 3 | RGB 三通道 |
| H（高度） | 640 | 图片高 640 像素 |
| W（宽度） | 640 | 图片宽 640 像素 |

**两个排列方式的区别（非常重要）：**

- **NHWC**：`[批次, 高, 宽, 通道]` —— 像素的 RGB 三个值连续存放，OpenCV 默认用这种
- **NCHW**：`[批次, 通道, 高, 宽]` —— 先把所有红色像素排一起，再所有绿色，再所有蓝色

你的模型用的是 NCHW，但 OpenCV 的 `cv::Mat` 默认是 HWC 排列（交错存储）。所以预处理后还需要"拆通道"把 HWC 变成 CHW——这个后面 Preprocess 里会详细讲。

### ⑤ 分配输入缓冲区

```cpp
// 一个 640x640 的 RGB 图（所有像素初始化为灰色 114,114,114）
input_rgb_ = cv::Mat(input_height_, input_width_, CV_8UC3,
                     cv::Scalar(114, 114, 114));

// 如果模型是 NCHW 格式，额外分配一个"平面排列"的缓冲区
if (input_is_nchw_) {
  const size_t plane = input_width_ * input_height_;  // 一个通道的像素数
  input_chw_.resize(plane * 3);                       // 三通道总大小

  // 用三个 cv::Mat "视图"分别指向 R、G、B 平面
  input_chw_views_[0] = cv::Mat(input_height_, input_width_, CV_8UC1,
                                input_chw_.data() + plane * 0);  // R 通道
  input_chw_views_[1] = cv::Mat(input_height_, input_width_, CV_8UC1,
                                input_chw_.data() + plane * 1);  // G 通道
  input_chw_views_[2] = cv::Mat(input_height_, input_width_, CV_8UC1,
                                input_chw_.data() + plane * 2);  // B 通道
}
```

这里有个巧妙的设计：`input_chw_views_` 三个 `cv::Mat` 并不是各自独立分配内存，而是"视图"（view）——它们共享同一块 `input_chw_` 内存，只是指向不同的偏移位置。这样 `cv::split()` 拆分通道时可以直接写进去，不需要额外拷贝。

### ⑥ RGA 硬件初始化——尝试，失败就降级

```cpp
std::string rga_error;
if (rga_preprocessor_.Init(input_width_, input_height_, &rga_error)) {
  std::cout << "RGA hardware acceleration initialized\n";
} else {
  std::cout << "RGA init failed (" << rga_error << "), using OpenCV fallback\n";
}
```

RGA 是 Rockchip 的硬件加速模块，能做图像缩放和色彩转换。这里只是尝试初始化。如果失败了（比如板子上的 `/dev/rga` 设备不存在），程序不会崩溃——它自动降级用 OpenCV。下一章会专门讲 RGA。

---

## 5. Preprocess 函数：画面预处理双路径

`Preprocess()` 的作用是把摄像头拍到的 640x480 BGR 画面，变成 NPU 要的 640x640 RGB 画面。

**为什么需要预处理？**
- 摄像头画面是 **BGR** 排列（OpenCV 默认），模型要的是 **RGB**
- 摄像头是 640x480（比例 4:3），模型输入是 640x640（正方形）
- 如果模型用 NCHW，还要把数据从 HWC 排列转成 CHW 排列

**处理流程：**

```
BGR 640x480
    │
    ├── 路径A：RGA硬件（快6倍）
    │   resize + letterbox + BGR2RGB 一步完成
    │
    └── 路径B：OpenCV软件（兼容性最好）
        cv::resize → cv::cvtColor(BGR→RGB) → letterbox padding
    │
    ▼
RGB 640x640 (HWC交错排列)
    │
    └── 如果需要 NCHW：
        cv::split() 拆成三个独立通道 → input_chw_ (CHW平面排列)
    │
    ▼
送入 NPU
```

### 代码结构（简化）

```cpp
void RknnDetector::Preprocess(const cv::Mat& frame, PreprocessMeta* meta) {
  // 尝试 RGA 硬件路径
  if (rga_preprocessor_.IsAvailable()) {
    float scale; int pad_x, pad_y; std::string rga_error;
    if (rga_preprocessor_.Process(frame, &input_rgb_, &scale, &pad_x, &pad_y, &rga_error)) {
      // RGA 成功！只需要再拆通道（如果需要NCHW的话）
      if (input_is_nchw_) {
        cv::split(input_rgb_, input_chw_views_);
      }
      // 记录缩放和padding参数，推理时还原坐标要用
      meta->scale = scale;
      meta->pad_x = pad_x; meta->pad_y = pad_y;
      return;  // 搞定，走人
    }
    // RGA 失败了，掉到下面的 OpenCV 路径
  }

  // OpenCV 软件路径（第 393 行开始）
  // ① 计算 letterbox 参数
  const float scale = std::min(
      float(input_width_)  / frame_w,    // 640 / 640 = 1.0
      float(input_height_) / frame_h);   // 640 / 480 = 1.333
  const int new_w = round(frame_w * scale);   // 640 * 1.0 = 640
  const int new_h = round(frame_h * scale);   // 480 * 1.0 = 480
  const int pad_x = (640 - 640) / 2 = 0;      // 没有水平padding
  const int pad_y = (640 - 480) / 2 = 80;     // 上下各80像素灰色条

  // ② 把输入区刷成灰色（114,114,114 是 YOLO 训练时的标准填充色）
  input_rgb_.setTo(cv::Scalar(114, 114, 114));

  // ③ 缩放 + 色彩转换，贴到 input_rgb_ 指定位置
  cv::resize(frame, resized_bgr_, cv::Size(640, 480));
  cv::cvtColor(resized_bgr_,
               input_rgb_(cv::Rect(pad_x, pad_y, 640, 480)),  // 贴到中间区域
               cv::COLOR_BGR2RGB);

  // ④ 如果模型要 NCHW —— 拆通道
  if (input_is_nchw_) {
    cv::split(input_rgb_, input_chw_views_);
  }
}
```

### letterbox 是什么意思？

你的摄像头是 4:3（640x480），模型要正方形（640x640）。怎么办？

**letterbox = 保持原图比例缩放，不够的地方填灰色。**

```
┌─────────────────────┐
│   ░░░░░░░░░░░░░░░   │  ← 灰色填充（上）
│   ░░░░░░░░░░░░░░░   │
├─────────────────────┤
│                     │
│    实际画面内容      │  ← 480像素高
│    （保持比例）      │
│                     │
├─────────────────────┤
│   ░░░░░░░░░░░░░░░   │  ← 灰色填充（下）
│   ░░░░░░░░░░░░░░░   │
└─────────────────────┘
    ←—— 640像素宽 ——→
```

为什么填 114？因为 YOLO 官方在训练时就是用 114 填充的。你的推理必须和训练保持一致。

### NCHW vs NHWC：为什么需要拆通道

OpenCV 的 `cv::Mat` 存储格式是 **HWC**（高度 → 宽度 → 通道），也就是像素的三个色值连续放在一起：

```
HWC排列（OpenCV默认）：
像素(0,0)的B 像素(0,0)的G 像素(0,0)的R  像素(0,1)的B ...
```

NPU 要求 NCHW 格式（通道 → 高度 → 宽度），也就是所有 R 值排一起，然后所有 G，然后所有 B：

```
NCHW排列（NPU要的）：
R(0,0) R(0,1) R(0,2) ... 所有红色像素 ...  G(0,0) G(0,1) ... 所有绿色 ...  B(0,0) ...
```

`cv::split(input_rgb_, input_chw_views_)` 就是把交错排列的 RGB 图拆成三个独立的平面。`input_chw_views_[0]` 是 R 通道，`[1]` 是 G，`[2]` 是 B，它们在内存里是连续的。

---

## 6. Infer 函数：一步一步走进 NPU 推理

`Infer()` 是项目的灵魂函数。我们用"分镜头叙事"的方式一步步拆开看。

```cpp
std::optional<Detection> RknnDetector::Infer(const cv::Mat& frame,
                                             const std::optional<Box>& prior,
                                             InferTimings* timings,
                                             std::string* error) {
```

参数说明：
- `frame`：摄像头的一帧画面，BGR 格式
- `prior`：先验框（可能为空）。什么是先验框？就是你上一帧检测到的位置。把它传进来，解码时可以从检测结果里挑一个跟它最接近的——避免"跳变"
- `timings`：输出各阶段耗时统计
- `error`：输出错误信息

返回值是 `std::optional<Detection>`——可能检测到了，也可能没检测到。

### 阶段一：Preprocess（预处理）

```cpp
PreprocessMeta meta;
Preprocess(frame, &meta);  // 上面第5节刚讲过
```

### 阶段二：rknn_inputs_set —— 喂数据

```cpp
rknn_input input{};
input.index = 0;                 // 第0号输入（因为模型只有一个输入）
input.type = RKNN_TENSOR_UINT8;  // 数据类型：8位无符号整数
input.size = input_is_nchw_ ? input_chw_.size()
                            : input_rgb_.total() * input_rgb_.elemSize();
input.fmt = input_is_nchw_ ? RKNN_TENSOR_NCHW : RKNN_TENSOR_NHWC;
input.pass_through = 0;          // 不做透传
input.buf = input_is_nchw_ ? static_cast<void*>(input_chw_.data())
                           : static_cast<void*>(input_rgb_.data);

int ret = rknn_inputs_set(context_, 1, &input);  // 设置1个输入
```

关键字段解释：
- **`type`**：告诉 NPU 数据的原始类型。你的模型是 INT8 量化模型（文件名里有 `int8`），所以输入也是 UINT8
- **`fmt`**：数据排列格式。根据模型要求选 NCHW 或 NHWC
- **`buf`**：指向实际数据的指针
- **`pass_through`**：如果设为 1，数据会绕过 NPU 预处理直接进推理。你设 0，表示自己已经预处理好了

### 阶段三：rknn_run —— 执行推理（核心中的核心）

```cpp
ret = rknn_run(context_, nullptr);
```

就一行。这是让 NPU 真干活的地方。第二个参数 `nullptr` 表示不需要扩展参数。

这一行可能是整个程序里最耗时但也是最"黑盒"的——你不需要知道 NPU 内部怎么算的，只需要知道给输入、拿输出。模型在编译成 `.rknn` 时已经固化了全部计算图。

### 阶段四：rknn_outputs_get —— 拿结果

```cpp
std::vector<rknn_output> outputs(io_num_.n_output);  // n_output 是输出张量个数
for (uint32_t i = 0; i < io_num_.n_output; ++i) {
  outputs[i].want_float = 0;    // 不要自动转浮点，我们自己处理
  outputs[i].is_prealloc = 0;   // 不预分配，让API自己分配
}

ret = rknn_outputs_get(context_, io_num_.n_output, outputs.data(), nullptr);
```

YOLO 模型通常有 3 个输出（对应三个检测尺度——大目标、中目标、小目标），每个输出是一个 NxM 的矩阵。

NPU 输出的原始数据是量化后的整数（INT8），你要在下一步把它转回浮点数。

**别忘了释放输出缓冲区**——在函数末尾：
```cpp
(void)rknn_outputs_release(context_, io_num_.n_output, outputs.data());
```
不释放会内存泄漏。

### 阶段五：ToMatrix —— 张量数据转浮点矩阵

```cpp
std::vector<OutputMatrix> matrices;
for (uint32_t i = 0; i < io_num_.n_output; ++i) {
  matrices.push_back(ToMatrix(output_attrs_[i], outputs[i]));
}
```

`ToMatrix` 的核心职能是把 NPU 输出的**原始字节**根据数据类型转成**浮点数**。你的模型是 INT8 量化的，所以数据要反量化：

```
float_value = (int8_value - zero_point) * scale
```

代码里处理了四种类型：

```cpp
switch (attr.type) {
  case RKNN_TENSOR_FLOAT32:  // 本来就已经是浮点，直接拷
    std::copy(src, src + count, matrix.data.begin());
    break;

  case RKNN_TENSOR_FLOAT16:  // 半精度浮点 → 转成单精度浮点
    for (...) { matrix.data[i] = HalfToFloat(src[i]); }
    break;

  case RKNN_TENSOR_INT8:    // INT8量化 → 反量化
    for (...) { matrix.data[i] = (src[i] - zp) * scale; }
    break;

  case RKNN_TENSOR_UINT8:   // UINT8量化 → 反量化
    for (...) { matrix.data[i] = (src[i] - zp) * scale; }
    break;
}
```

`HalfToFloat` 是用 IEEE 754 标准手动把 16 位半精度浮点数转成 32 位单精度——因为 C++ 标准库的 `std::half` 不一定处处可用。

`ToMatrix` 还做了另一件事：**如果矩阵行少列多，就转置**，让数据变成"每行是一个检测结果"的格式，方便后面解码。

### 阶段六：DecodeOutputs —— 把 NPU 的原始数字翻译成检测结果

NPU 输出的每一行代表一个"潜在的检测框"，格式是：

```
[cx, cy, w, h, ...类别置信度...]
```

- `cx, cy`：框的中心点坐标（可能是归一化到 0~1 的相对坐标，也可能是像素坐标）
- `w, h`：框的宽高
- 后面的数字：每个类别的置信度（这个框属于"目标"的概率）

**解码逻辑**（简化）：

```cpp
for (每一行) {
  float cx = 第0个数字;
  float cy = 第1个数字;
  float w  = 第2个数字;
  float h  = 第3个数字;

  // 如果坐标范围在 0~2 之间，说明是归一化的，放大到输入尺寸
  if (最大值 <= 2.0f) {
    cx *= input_width_;   // 乘 640
    cy *= input_height_;  // 乘 640
    w  *= input_width_;
    h  *= input_height_;
  }

  // 从类别分里找置信度最高的
  int   best_class = -1;
  float best_score = 0.0f;
  for (int i = 0; i < class_count; ++i) {
    if (第(4+i)个数字 > best_score) {
      best_score = 第(4+i)个数字;
      best_class = i;
    }
  }

  // 过滤低置信度
  if (best_score < config_.conf_threshold) continue;  // 默认0.45

  // 把输入空间坐标映射回原始帧坐标
  box.x1 = (cx - w/2 - meta.pad_x) / meta.scale;
  box.y1 = (cy - h/2 - meta.pad_y) / meta.scale;
  box.x2 = (cx + w/2 - meta.pad_x) / meta.scale;
  box.y2 = (cy + h/2 - meta.pad_y) / meta.scale;

  // 裁剪到画面范围内
  box = ClipBox(box, meta.frame_w, meta.frame_h);

  // 过滤太小的框
  if (box宽度 < config_.min_box_size || box高度 < config_.min_box_size) continue;

  detections.push_back(Detection{box, 中心点, 置信度, best_class});
}
```

**关键坐标变换**：

NPU 输出的坐标是在"被缩放和 padding 后的输入空间"（640x640）里的。你要把它映射回"原始帧空间"（640x480）：

```
原始坐标 = (NPU坐标 - padding偏移) / 缩放比例

例子：
NPU 说中心点在 (320, 320)，pad_x=0, pad_y=80, scale=1.0
原始 y = (320 - 80) / 1.0 = 240  ← 正确！因为在 640x640 正方形里，
                                     有效画面只有中间480像素高
```

### NMS（非极大值抑制）

解码后可能有很多框重叠在同一个目标上。NMS 的作用是去重：

```cpp
// 按置信度从高到低排序
std::sort(detections.begin(), detections.end(),
          [](const Detection& a, const Detection& b) { return a.conf > b.conf; });

// 从最高分开始，保留它，然后删除所有和它 IOU 超过阈值的框
for (每个检测框) {
  bool keep = true;
  for (已保留的框) {
    if (两个框的IOU > config_.iou_threshold) {  // 默认0.50
      keep = false;  // 重叠太大，丢弃
      break;
    }
  }
  if (keep) kept.push_back(这个框);
}
```

IOU（Intersection over Union）是两个框的交集面积除以并集面积。如果 IOU > 0.5，说明两个框基本上在指同一个东西。

### ChooseDetection：从检测结果里挑一个"最好的"

如果只有一个检测结果，直接返回它。如果有先验框（上一帧的位置），就从多个检测中挑一个跟先验框最接近的：

```cpp
std::optional<Detection> ChooseDetection(
    const std::vector<Detection>& detections,
    const std::optional<Box>& prior, ...) {

  if (detections.empty()) return std::nullopt;       // 没检测到
  if (!prior.has_value()) return detections.front(); // 没有先验，返回置信度最高的

  // 有先验框：综合打分
  for (每个检测框) {
    float score = det.conf;                          // 置信度（基础分）
    score += 0.35 * max(0, 1 - 距离/对角线 * 3);     // 距离加分（越近越好）
    score += 0.30 * BoxIou(det.box, prior);          // IOU加分（重合越多越好）
    // 挑总分最高的
  }
  return best;
}
```

权重设计的含义：
- **0.35 距离权重**：优先选离上一帧位置近的框（避免跳变）
- **0.30 IOU 权重**：优先选和上一帧重叠大的框（形状也接近）

为什么需要这个？因为在瞄准场景里，目标可能连续多帧被检测到，但每帧可能有多个候选框。用先验框做引导，可以让追踪更稳定——不会这一帧选左上角的框，下一帧选右下角的框。

---

## 7. DetectorWorker：把推理放到独立线程

你可能会问：为什么不直接在主循环里调 `Infer()`？因为推理要花时间（大约 5~10ms），如果放在主循环里，主循环就会被阻塞——摄像头帧来不及读，控制指令发不出去。

**解决办法**：开一个独立线程专门做推理。主循环只管"提交任务"，推理线程管"执行"。

### 类的核心成员

```cpp
class DetectorWorker {
  std::unique_ptr<RknnDetector> detector_;  // 持有推理器（独占所有权）
  std::thread thread_;                      // 工作线程
  std::atomic<bool> stop_{false};           // 停止标志（原子变量）
  std::atomic<bool> started_{false};        // 启动标志

  mutable std::mutex mutex_;                // 互斥锁
  bool busy_ = false;                       // 推理中？
  bool pending_ = false;                    // 有待处理的任务？
  cv::Mat pending_frame_;                   // 待处理的帧
  std::optional<Box> pending_prior_;        // 待处理的先验框

  InferResult latest_result_;               // 最新推理结果
  double smoothed_infer_fps_ = 0.0;         // 平滑后的推理帧率
};
```

### 生命周期：Start / Stop

```cpp
void Start() {
  if (started_.exchange(true)) return;       // 已经启动了，忽略
  stop_.store(false);
  thread_ = std::thread(&DetectorWorker::Run, this);  // 创建线程，执行 Run()
}

void Stop() {
  stop_.store(true);                         // 通知线程停下
  if (thread_.joinable()) thread_.join();    // 等线程结束
  started_.store(false);
}
```

`std::atomic<bool>::exchange(true)` 是一个原子操作：把变量设为 true，同时返回旧值。如果旧值已经是 true，说明 Start 已经被调用过了，直接返回。

### 异步接口：Submit / Busy / LatestResult

这三个函数是主循环和推理线程之间的"通信协议"：

```cpp
// 主循环调用：提交一帧给推理线程
bool DetectorWorker::Submit(const cv::Mat& frame, const std::optional<Box>& prior) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (busy_ || pending_) return false;  // 推理线程忙，拒绝（丢掉这帧）
  pending_frame_ = frame;
  pending_prior_ = prior;
  pending_ = true;
  return true;
}

// 主循环调用：问线程忙不忙
bool DetectorWorker::Busy() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return busy_ || pending_;
}

// 主循环调用：拿最新的推理结果
InferResult DetectorWorker::LatestResult() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_result_;  // 拷贝一份返回
}
```

`std::lock_guard` 在构造时自动 lock，出作用域自动 unlock——你永远不会忘了解锁。

### 主循环：Run()

```cpp
void DetectorWorker::Run() {
  while (!stop_.load()) {                    // 只要没被叫停
    cv::Mat frame;
    std::optional<Box> prior;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (pending_) {                        // 有待处理的任务
        pending_ = false;
        busy_ = true;
        frame = pending_frame_;              // 拿走待处理帧
        prior = pending_prior_;
        pending_frame_.release();            // 释放引用
      }
    }

    if (frame.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));  // 没任务，睡1ms
      continue;
    }

    // 执行推理！
    auto detection = detector_->Infer(frame, prior, &timings, &error);

    // 平滑帧率
    double inst_fps = 1.0 / (end - start);
    smoothed_infer_fps_ = SmoothValue(smoothed_infer_fps_, inst_fps, 0.15);

    // 更新结果
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++sequence_;
      latest_result_.seq = sequence_;
      latest_result_.detection = detection;
      latest_result_.infer_fps = smoothed_infer_fps_;
      busy_ = false;                         // 干完活了
    }
  }
}
```

**帧率平滑**用的是一种叫"指数移动平均"的技巧：

```cpp
// types.hpp
inline double SmoothValue(double previous, double current, double alpha) {
  return previous <= 0.0 ? current : previous * (1.0 - alpha) + current * alpha;
}
// alpha=0.15 表示新值权重15%，旧值权重85%
// 这样显示的帧率不会因为一两帧的波动而跳来跳去
```

---

## 8. 关键设计点总结

### 8.1 为什么用异步 Worker 而不是同步推理？

```
同步（一次一帧）：
摄像头 → 推理 → 发指令 → 摄像头 → 推理 → ...
         ↑ 这5-10ms主循环卡住，摄像头帧堆积

异步（流水线）：
摄像头线程：┌─读帧1─┐  ┌─读帧2─┐  ┌─读帧3─┐ ...
推理线程：         └─推理1─┘  └─推理2─┘  └─推理3─┘ ...
主循环：    ┌─取结果1─┐ ┌─发指令─┐ ┌─取结果2─┐ ...
```

摄像头采集、AI 推理、控制发送三条流水线各自独立，互不阻塞。

### 8.2 为什么用 `pending_` 标志跳过旧帧？

```
如果推理线程正在忙，主循环又提交了一帧：

方案A（不用 pending）：排队。但队列会越来越长，
     推理追不上采集速度 → 处理全是"过时的帧"
方案B（用 pending）：pending_=true 时拒绝 Submit，
     丢掉中间帧，只推理最新的一帧
```

你的项目用的是方案B——**只处理最新帧**。这个设计对实时瞄准很关键：推理结果滞后 1~2 帧可以接受，但推理想处理 10 帧前的数据就不对了。

### 8.3 `Warmup` —— 提前热身

```cpp
bool RknnDetector::Warmup(int count, std::string* error) {
  cv::Mat dummy(640, 640, CV_8UC3, cv::Scalar(114, 114, 114));  // 灰色假图
  for (int i = 0; i < count; ++i) {
    Infer(dummy, std::nullopt, &timings, error);  // 跑几遍预热
  }
}
```

NPU 第一次推理通常比后续慢（因为要分配缓存、预热流水线）。所以程序启动时先用假图跑 3 遍"热身"，这样第一帧真推理就不是"冷启动"了。

---

## 章节小结

这一章你看到了整个项目最核心的模块。回顾要点：

| 步骤 | 干了什么 | 关键 API |
|------|---------|---------|
| LoadModel | 读 .rknn 文件进内存 | `std::ifstream` + `binary` |
| rknn_init | 初始化 NPU | `rknn_init()` |
| core_mask | 指定用几个核心 | `rknn_set_core_mask()` |
| QueryTensors | 弄清输入输出格式 | `rknn_query()` |
| Preprocess | 缩放/letterbox/BGR→RGB/NCHW | RGA 或 OpenCV |
| rknn_inputs_set | 喂数据 | `rknn_inputs_set()` |
| rknn_run | 执行推理 | `rknn_run()` |
| rknn_outputs_get | 拿结果 | `rknn_outputs_get()` |
| ToMatrix | 量化数据 → 浮点数 | 反量化 + 转置 |
| DecodeOutputs | 原始数字 → 检测框 | 坐标映射 + NMS |
| ChooseDetection | 选最佳框 | 距离加权 + IOU 加权 |
| DetectorWorker | 异步推理线程 | `std::thread` + `std::mutex` |

搞懂了这一章，你就理解了项目的"心脏"是怎么跳动的。下一章专门讲 RGA 硬件加速——让预处理从 2ms 降到 0.3ms 的秘密武器。
