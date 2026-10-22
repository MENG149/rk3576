# RK3576 C++ RKNN Targeting 总览教学文档

> **文档定位**：从一张已训练模型文件出发,沿着 `rknn` → `rga` → `cv::Mat` → `Kalman` → `homography` → `serial` 的完整数据链路,把本仓库每一行 C++ 代码、每一个 CMake 开关、每一项板端调优动作,都讲清楚。
>
> **目标读者**：要把这份代码"熟透"的工程师,以及接手后续 120 Hz 升级工作的同学。
>
> **基线日期**：2026-07-14。涉及到的所有数字、性能基线、bug 修复记录在 2026-07-12 报告与 2026-07-13 设计中已有出处,本文件不再重复堆数字,而是讲**为什么是这个数字**。
>
> **与已有教程的关系**：`docs/learn-cpp/` 已经按代码模块拆出 13 章入门教程,`docs/superpowers/specs/2026-07-13-...design.md` 是 120 Hz 升级的设计草案。本文是**纵向全集**:不讲"每个文件怎么写",而讲"每个文件为什么这么写、它和上下游怎么耦合"。

---

## 0. 一句话看懂这个项目

`rkfast_target` 是个**板端单进程视觉闭环**:每 36 ms(27.5 FPS 摄像头硬件上限)从 `/dev/video73` 拉一帧 1280×720 MJPG → 通过 **RGA 把图像 resize + letterbox + BGR→RGB 一次性硬写到 512×512 NHWC FP16 输入缓冲** → **NPU 双核 FP16 RKNN** 跑一阶段矩形检测 → **OpenCV 在 ROI 内找四边形** 做亚像素角点精修 → **6 维 Kalman** 把上一帧状态、当前 ROI 测量、当前 RKNN 测量三者融合,并按 50 ms lead-time 外推下一帧瞄准点 → **解算图像到 A4 靶面厘米坐标的单应矩阵** → **按 50 Hz 把 12 字节二进制协议(/dev/ttyS4)送给 STM32 二维云台**,同时以 **5 Hz** 打印 `$AIM` 调试行;另开一个 MJPEG HTTP 服务,便于远程看带标注的预览流。

下面将这段流程拆成 26 节,逐节讲透。

---

## 1. 仓库目录与文件清单

```
rk3576_rknn_targeting/
├─ README.md                                # 项目门面:验证结论 + 编译/运行步骤
├─ docs/                                    # 所有文档
│  ├─ README.md                             # docs 的索引
│  ├─ learn-cpp/00..13-...md                # 13 章按模块逐文件教学(已存在)
│  ├─ guides/board-deployment.md            # 板端部署/排错
│  ├─ reference/runtime-files.md            # 当前运行文件清单
│  ├─ reports/RK3576_矩形识别修复与性能优化报告_2026-07-12.md  # 性能调优报告
│  ├─ superpowers/
│  │  ├─ specs/...
│  │  │  ├─ 2026-05-26-rk3576-cpp-fast-targeting-design.md   # 最初 C++ 设计
│  │  │  ├─ 2026-05-27-cpp-rknn-core-mask-test-design.md     # NPU 双核尝试
│  │  │  ├─ 2026-06-02-rect-accuracy-coord-velocity-design.md # 坐标与速度口径
│  │  │  ├─ 2026-07-12-cpp-project-cleanup-design.md         # 目录重整设计
│  │  │  ├─ 2026-07-12-rknn-input-zero-copy-design.md        # FP16 zero-copy 设计
│  │  │  ├─ 2026-07-13-rk3576-120hz-vision-tracking-design.md # 120Hz 升级设计
│  │  │  └─ 2026-07-14-rk3576-cpp-master-teaching.md         # ← 本文
│  │  └─ plans/2026-07-12-cpp-project-cleanup.md             # 实施计划
│  └─ archive/
│     ├─ python-runtime/                    # 旧 Python 实现(不再维护)
│     └─ legacy/                            # 旧文档(INT8、CMake 错误修复等)
├─ cpp_fast_targeting/                      # 唯一当前活跃的 C++ 工程
│  ├─ CMakeLists.txt                        # 全文 build 系统
│  ├─ build_and_verify.sh                   # 板上构建+自检脚本(10 步)
│  ├─ run_camera.sh                         # 板上启动脚本(默认参数固化)
│  ├─ benchmark.sh                          # 压测脚本
│  ├─ include/rkfast/
│  │  ├─ aim_writer.hpp                     # AIMWriter 类声明
│  │  ├─ camera_source.hpp                  # CameraSource 类声明
│  │  ├─ config.hpp                         # AppConfig + ParseCommandLine 声明
│  │  ├─ mjpeg_server.hpp                   # MjpegServer 类声明
│  │  ├─ rect_refiner.hpp                   # RectRefiner 类声明
│  │  ├─ rga_preprocess.hpp                 # RgaPreprocessor 类声明
│  │  ├─ rknn_detector.hpp                  # RknnDetector + DetectorWorker 类声明
│  │  ├─ target_plane.hpp                   # TargetPlane 类声明
│  │  ├─ thread_affinity.hpp                # BindCurrentThread 声明
│  │  ├─ tracker.hpp                        # RectTracker 类声明
│  │  └─ types.hpp                          # 全文基础类型 + 几何工具
│  ├─ src/
│  │  ├─ aim_writer.cpp                     # 串口 12 字节二进制 + ASCII $AIM
│  │  ├─ camera_source.cpp                  # V4L2/OpenCV 异步采集
│  │  ├─ config.cpp                         # 命令行解析
│  │  ├─ main.cpp                           # 主循环 + 调试 overlay
│  │  ├─ mjpeg_server.cpp                   # multipart/x-mixed-replace HTTP
│  │  ├─ rect_refiner.cpp                   # ROI 内找四边形 + 评分
│  │  ├─ rga_preprocess.cpp                 # im2d resize+letterbox+BGR2RGB
│  │  ├─ rknn_detector.cpp                  # 模型加载 / 推理 / 解码 / NMS
│  │  ├─ target_plane.cpp                   # 单应矩阵 + 厘米坐标
│  │  ├─ thread_affinity.cpp                # pthread_setaffinity_np 包装
│  │  └─ tracker.cpp                        # 6 维 Kalman + 状态机
│  ├─ tests/CMakeLists.txt                  # 单元测试子项目
│  ├─ tests/rkfast_tests.cpp                # BoxIou/Refiner/Tracker 单测
│  ├─ tools/image_probe.cpp                 # 静态图诊断工具(不依赖摄像头)
│  ├─ tools/input_mode_compare.cpp          # auto/legacy/zero-copy 对比
│  ├─ third_party/rknn/include/rknn_api.h   # librknnrt 头文件(2.3.2)
│  └─ build/                                # cmake 输出(不在仓库内)
├─ models/
│  └─ best_n_stage1_512_fp16.rknn           # 唯一运行模型(FP16 512×512)
└─ tools/
   ├─ rk3576_monitor.sh                     # 板端状态观测
   ├─ rk3576_performance.sh                 # CPU/NPU/DDR → performance
   └─ test_rga.sh                           # RGA 设备裸测
```

> **重要原则**:仓库只保留"已验证"的运行版本。Python runtime 与 INT8 量化模型都被移到 `archive/`,绝不能直接照搬。本文件之后讨论的所有结论都是基于 `models/best_n_stage1_512_fp16.rknn` 这唯一一个模型。

---

## 2. 模型训练与导出合同

### 2.1 训练在哪做

本仓库**不包含训练代码**。模型训练在外部 GPU 工作站(常用 Ultralytics 家族的 YOLO 变种)上完成,产物是 PyTorch/ONNX,然后用 Rockchip 的 `rknn-toolkit2` 在 aarch64 板端的 Python 环境下导出为 `.rknn`。这一段历史数据在 `archive/legacy/模型文件问题修复.md` 与 `archive/python-runtime/RUN.md` 中可追,本仓库不再维护。

### 2.2 推理合同(`rknn_detector.cpp` 期望的模型格式)

`RknnDetector::QueryTensors` 在加载后立即跑一次 `RKNN_QUERY_IN_OUT_NUM` + `RKNN_QUERY_INPUT_ATTR` + `RKNN_QUERY_OUTPUT_ATTR`,把模型假设"硬化"到代码里。**只要换一个不同拓扑的模型,这里立刻失败或后果难以预测**。具体合同:

| 项 | 期望 | 来源代码 |
|---|---|---|
| 输入数量 | 恰好 1 个 | `io_num_.n_input != 1` 报错 |
| 输入 shape | `[1, H, W, 3]` 或 `[1, 3, H, W]` | `dims[1..3]` |
| 输入 w/h | 当前为 `512×512` | `input_width_/input_height_` |
| 输入通道 | 3 (RGB/BGR 顺序取决于转换路径) | `input_channels_ != 3` 报错 |
| 量化类型 | **FP16**(INT8 已确认 score=0) | README/models/ |
| 每个输出行 | `[cx, cy, w, h, ...cls...]`,长度 ≥ 5 | `DecodeOutputs` 第 4 起读 |
| 输出分数口径 | `class_count==1` 时直接 `item[4]`;`>1` 时 `objectness × item[5+i]` | `DecodeOutputs` 分支 |
| 输出是否归一化 | 如果行任何分量绝对值 ≤ 2,被视为归一化坐标,乘以 `input_w/h` | `DecodeOutputs` 启发式 |

`config.hpp` 默认:

```cpp
model_path = "../../models/best_n_stage1_512_fp16.rknn";
input_size = 512;
class_count = 1;
target_class = 0;
conf_threshold = 0.25f;
iou_threshold = 0.50f;
max_det = 1;
```

> 一旦要换模型,不只是换路径 —— `input_size / class_count / target_class / conf / iou` 都要重新标定;`conf_threshold` 不是凭空挑的,2026-07-12 报告里测得这张 FP16 模型在 20 张训练图上 conf 0.87~0.94,0.25 是相对保守的"几乎不会漏掉"的下界,恰好也是"几乎不会误触发"的上界之一。

### 2.3 把模型传给 NPU:`LoadModel` 全链路

1. `std::ifstream` 以二进制方式把整个 `.rknn` 读到 `model_data_`(`vector<uint8_t>`)。
2. `rknn_init(&context_, model_data_.data(), size, 0, nullptr)` 在 RKNN 驱动中创建上下文。
3. `RKFAST_RKNN_HAS_CORE_MASK` 编译期探测(见第 3 节),如果可用就 `rknn_set_core_mask(ctx, 01)` 启双核。
4. `QueryTensors` 拿到输入输出元信息,填好 `input_width_/height_/channels_` 和 `input_is_nchw_`。
5. 根据 `rknn_input_mode="auto"`,尝试 `SetupZeroCopyInput`(FP16 pass-through);失败自动降级 `legacy`(UINT8 + `rknn_inputs_set`)。
6. 最后 `Init` 一份 `RgaPreprocessor`,即使 RGA 不可用也不致命,后面 `Preprocess` 自动回退 OpenCV。

### 2.4 量化"陷阱"

> 2026-07-12 实测结论:`best_n_stage1_512_int8.rknn` **在全部 20 张真实训练图上 score 恒为 0.0**,不是阈值问题,是导出彻底失败。罪魁推断是 `model.2.m.0.cv2.conv.weight`、`model.23.cv3.2.1.1.0.conv.weight` 两个 outlier 权重层量化异常。**`config.hpp` 把 INT8 路径封死为不提供,就是把"理论上可以接受 INT8 加速但实际上模型废掉"这件事彻底挡在配置层**。

所以目前的训练产物合同是:**只允许交付 FP16**。

---

## 3. CMake 编译体系(全文 build 逻辑)

`cpp_fast_targeting/CMakeLists.txt` 是项目唯一的 build 系统。下面按出现顺序讲每一段:

### 3.1 基本面

```
cmake_minimum_required(VERSION 3.16)
project(rkfast_targeting LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)             # C++17 不可少(std::optional / structured bindings)
option(RKFAST_USE_RGA "Enable RGA hardware acceleration" ON)   # 默认开
find_package(Threads REQUIRED)         # pthreads
```

### 3.2 OpenCV 两种发现路径

先尝试现代的 `find_package(OpenCV ... COMPONENTS core imgproc video videoio highgui)`;板上环境失败时降级到 `pkg_check_modules(OPENCV4 REQUIRED IMPORTED_TARGET opencv4)`。最后统一成 `RKFAST_OPENCV_INCLUDE_DIRS / RKFAST_OPENCV_LIBS / RKFAST_OPENCV_TARGET` 三组变量,后面 `target_*` 二选一使用。

### 3.3 librknnrt + rknn_api.h

`find_path` 在多处搜索 `rknn_api.h`:

```
${CMAKE_CURRENT_SOURCE_DIR}/third_party/rknn/include   ← 项目本地(2.3.2)
/usr/include
/usr/local/include
/usr/include/rknn
/usr/include/librknn_api
/usr/include/rknpu2
```

`find_library` 在多处搜索 `librknnrt` 或 `rknnrt`。任一缺失就 `FATAL_ERROR`,不会让你跑在没有 NPU 库的环境里。

### 3.4 librga 可选特性

只有当 `RKFAST_USE_RGA=ON` 才找,找不到就仅 `WARNING`,不会失败 —— 因为 RGA 是性能优化项,不是功能正确性项。找到后把头文件目录和 `librga.so` 加进 `target_link_libraries`(用生成器表达式 `$<$<BOOL:${RKFAST_HAS_RGA}>:${RGA_LIB}>` 写条件链接),同时定义 `RKFAST_USE_RGA=1` 让 `rga_preprocess.cpp` 中的 `#ifdef` 拿到实现。

### 3.5 编译期能力探测:`rknn_set_core_mask` 是否存在

```
check_cxx_source_compiles("
  #include <rknn_api.h>
  int main() { rknn_set_core_mask(ctx, RKNN_NPU_CORE_0_1); }
" RKFAST_RKNN_HAS_CORE_MASK)
```

如果头文件存在但库不是 2.3+ 没有这符号,编译就报错,我们可以把它定义成 0,运行期 `--core-mask` 静默忽略(见 3.6 第 186 行 `RKFAST core_mask=default (API not available in this runtime)`)。这是"ABI 兼容"的实用写法,保证老库也能编。

### 3.6 三个可执行目标

| 目标 | 源文件 | 用途 |
|---|---|---|
| `rkfast_target` | 全部 11 个 src | 主程序 |
| `image_probe` | tools/image_probe.cpp + config/refiner/rga/rknn/thread_affinity | 静态图诊断,**不依赖摄像头** |
| `input_mode_compare` | tools/input_mode_compare.cpp + config/rga/rknn/thread_affinity | 三种 input mode 对比测量 |

主程序全部 `-Wall -Wextra -Wpedantic` + Release `-O3 -ffast-math -fno-math-errno -fno-trapping-math`。`-ffast-math` 在 ROI refine 里要注意:浮点 NaN 会被冲掉;但当前代码路径都先做 `std::isfinite` 检查,实际不影响正确性。

`check_cxx_source_compiles` 失败可能是因为链接测试调用了 `rknn_set_core_mask` 但库太老。`check_cxx_source_compiles` 默认只编译不链接,所以这里**只要头文件里有声明**就过 —— 这正好是我们要的语义,因为运行期才会真去链接符号。

### 3.7 关键编译开关

```bash
cmake -S cpp_fast_targeting -B cpp_fast_targeting/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DRKFAST_USE_RGA=ON
cmake --build cpp_fast_targeting/build --parallel "$(nproc)"
```

板上用 `bash build_and_verify.sh` 一步走完下面 10 步:
1. 架构检查(应是 aarch64)。
2. cmake / make / pkg-config 必备工具检查。
3. 旧 build 目录删除。
4. cmake 配置。
5. 解析 cmake log,确认 `RGA found:`。
6. 并行构建。
7. 二进制存在 + 可执行。
8. readelf 检查链上 `librga` + `librknnrt`。
9. `--help` 烟测。
10. ldd 检查无 missing libs。

板上任何一步失败都用红字 [FAIL],整体 exit 非 0,**这是给作者自己的"上线保险"**。

---

## 4. 数据类型 `types.hpp` 完全解析 —— 全部算法的"地基"

`types.hpp` 不依赖项目其他任何东西,纯粹是 POD + OpenCV 小工具。所有其他 `.cpp` 都用它。

### 4.1 核心结构

```cpp
struct Box { float x1,y1,x2,y2; };                              // 矩形(全画面坐标)
struct Detection { Box box; cv::Point2f center; float conf;
                   int class_id; int candidate_count; };        // 一条候选
struct RefinedRect { Box box; cv::Point2f center;
                     std::array<cv::Point2f,4> corners; float score; };
                                                                // OpenCV 精修出的四边形
struct FramePacket { cv::Mat frame;
                     double capture_done_ts; uint64_t seq; };   // 一帧 + 拍摄时间 + 帧号
struct RefineHints { std::optional<Box> prior_box;
                     float prior_aspect; float speed; };        // 跟踪器→精修器的提示
struct TrackSnapshot { /* 跟踪器对外的全部状态 */ };
struct InferTimings { /* 一次推理的 8 个分段时间 */ };
struct InferResult   { /* 一次推理的产出(检测 + 时间) */ };
struct VisualEstimate{ /* 一次发布的"视觉估计"对外契约 */ };
```

### 4.2 `FramePacket::capture_done_ts` 的命名学

命名故意叫"capture_done_ts"而不是"exposure_ts":OpenCV 的 V4L2 backend 在 aarch64 上**不暴露稳定的传感器曝光时间戳**,所以这个字段是 `cap.read()` 成功返回后立刻打的 `steady_clock`,**延迟约等于一帧的 USB/MIPI 解压时间**。这名字提醒下游:`tracker` 不能把它当 sensor 起点用。这是 2026-07-13 设计文档第 6.1 节反复强调的契约。

### 4.3 几何工具们

```cpp
inline float ClampFloat(v,lo,hi);                  // std::clamp 的 float 版
inline Box   ClipBox(b,w,h);                       // 限制到画面内,且强制 x2>x1 / y2>y1
inline float BoxWidth(b), BoxHeight(b), BoxArea(b);
inline cv::Point2f BoxCenter(b);                  // (x1+x2)/2, (y1+y2)/2
inline float BoxIou(a,b);                         // 常规 IoU,1e-6 防 0 除
inline double SmoothValue(prev,cur,alpha);        // 0 首帧直接采用 current
inline cv::Point2f CenterOffset(p,w,h);           // 画面中心为原点
inline cv::Point2f NormalizedOffset(p,w,h);       // 归一化到 [-1,+1]
```

注意三个**易错点**:

1. `ClipBox` 不会挑 `x2 > x1` 进来空盒,但**会把退化盒 swap 成合法**。下游必须自己检查宽高。
2. `BoxWidth/BoxHeight` 默认下限 1 px,避免在 Kalman size log 里出 `-inf`。
3. `BoxIou` 在两者不相交时返回 0,**不会返回负数**。

### 4.4 跟踪器 corner 顺序约定

`std::array<cv::Point2f, 4> corners` 在 `RectTracker::UpdateCorners` 和 `TargetPlane::Update` 里始终按下标组织:

```
0: top-left        1: top-right
2: bottom-right    3: bottom-left
```

这与 OpenCV `boundingRect.points()` 默认返回值相同,避免将来手动 reverse。`TargetPlane::OrderCorners` 在内部会重排一次保证这个顺序(以图像中心做极角排序 + SignedArea2 符号校正)。

### 4.5 时间口径

```cpp
inline double NowSeconds() {
  using Clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}
```

**全程只用 `steady_clock`**,不做 wall-clock 换算,也不换 `high_resolution_clock`。这意味着所有 `state_ts / last_measurement_ts / capture_done_ts` 都是**相对于进程启动的秒数**,可以靠 `SmoothValue` 跨线程、跨运行期对比。

### 4.6 `TrackerSnapshot::predicted_only`

```cpp
out.predicted_only = initialized_ &&
    state_timestamp_ - last_measurement_timestamp_ > kTimeEpsilon;
```

这是个**关键标志**:它告诉下游"这次发布的估计是基于 Kalman 外推的,而不是新的测量"。`main.cpp` 用它来 gate 是否要重新触发 RKNN 检测(避免漏掉已经丢失的目标)。

---

## 5. `main.cpp` 主循环 —— 数据集水的指挥棒

`main.cpp` 是把 8 个模块串成流水线的代码,也是后续 120 Hz 升级时改动最大的文件。完整流程如下:

### 5.1 启动阶段

```cpp
AppConfig config;
if (!ParseCommandLine(argc, argv, &config)) return 1;
cv::setNumThreads(max(1, config.opencv_threads));    // = 1,避免 OpenCV 内部线程和视觉线程抢核
cv::setUseOptimized(true);

if (!main_affinity 绑核失败) cerr << "...";
CameraSource camera(config);
if (!camera.Open(&error)) return 1;                 // 不会真阻塞;内部启动 capture 线程

RknnDetector detector(config);
if (!detector->Open(&error) || !detector->Warmup(N, &error)) return 1;
DetectorWorker worker(std::move(detector), config.detector_affinity);
worker.Start();                                       // 起后台推理线程

RectRefiner  refiner(config);
RectTracker  tracker(config);
TargetPlane  plane(config.target_width_cm, config.target_height_cm);
AimWriter    writer(config);  writer.Open();
MjpegServer  mjpeg;
if (mjpeg 端口>0) mjpeg.Start(端口,质量);
```

启动顺序的重要性:

- **先**绑 main 线程核,再起子线程,后面 `BindCurrentThread` 才能"在子线程入口"绑不同核。
- 摄像头开之前已经把参数(fourcc / width / fps / buffersize)塞好,确保 `Open` 完后就协商到目标格式。
- `Warmup` 跑 `warmup_runs` 次推理(默认 3)再启动 worker,避免首次推理的初始化开销污染统计。
- `MjpegServer` 即使开启也不消耗编码成本:它会在 `PushFrame` 里看 `client_count()==0` 直接 return。

### 5.2 每帧主循环

```cpp
while (true) {
  FramePacket packet;
  if (!camera.WaitLatest(last_frame_seq, &packet, 100ms)) continue;  // 等下一帧
  last_frame_seq = packet.seq;
  const double loop_start = NowSeconds();

  tracker.Predict(packet.capture_done_ts, packet.frame.size(), packet.seq); // 关键!
  InferResult result = worker.LatestResult();                              // 上次推理的产出
  bool new_result = (result.seq != 0 && result.seq != last_result_seq);
  if (new_result && result_age <= 50ms) 把 result.detection 投影到当前帧 → current_detection;

  // 选 guide box
  Box guide = current_detection? current_detection.box : tracker.PriorBox();
  TrackSnapshot prior = tracker.Snapshot();
  std::optional<cv::Point2f> ref = prior.active? prior.center : nullopt;

  // 1) OpenCV 精修
  std::optional<RefinedRect> refined = refiner.Refine(packet.frame, guide, ref, tracker.Hints());

  // 2) 把精修或裸检测送进 tracker
  bool accepted = false;
  if (refined) accepted = tracker.Update(refined, ..., "roi+rknn" or "roi");
  if (!accepted && current_detection) tracker.Update / Reacquire(current_detection, ...);

  // 3) 更新 target plane(只有 active + TRACKING + has_corners 才算)
  TrackSnapshot track = tracker.Snapshot();
  if (track.active && track.state != "ACQUIRING" && track.has_corners)
    target_plane.Update(track.corners, packet.frame.size(), track.state_ts, &err);
  else
    target_plane.Invalidate();
  VisualEstimate estimate = BuildVisualEstimate(track, plane);

  // 4) 是否重新提交 RKNN
  bool need_detection = !track.active || track.predicted_only || frames_until_detection<=0;
  if (need_detection) { worker.Submit(packet, tracker.PriorBox());
                        frames_until_detection = track.active? interval_locked : 0; }
  else --frames_until_detection;

  // 5) 输出 + 统计
  writer.Send(track, cols, rows);
  if (每 stats_interval) cout << "stats frame_fps=..." << endl;

  // 6) 调试 overlay
  if (show_window || mjpeg 有 client) {
     cv::Mat overlay = DrawDebug(packet.frame, track, estimate, ...);
     if (show_window) cv::imshow;   // 同时按 q 退出
     if (mjpeg 有 client) mjpeg.PushFrame(overlay);
  }
}
```

### 5.3 几个核心不变量

1. **`tracker.Predict` 永远先做**,把上一帧或上一拍的状态推到当前帧时间。这是"每帧一次"的统计意义。
2. **`worker.LatestResult()` 非阻塞,只读**。它从不等待 NPU;若当前没有新结果,`last_result_seq != result.seq` 也不进来。
3. **`refiner` 接受 guide_box = 当前检测 或 跟踪器先验**。没有检测时回退到 `tracker.PriorBox()`。
4. **`tracker.Update(RefinedRect)` 成功后顺手调 `UpdateCorners(EMA)`**,失败则保留原角;corner 大跳(> `corner_reset_dist`)时放弃 EMA,直接覆盖。
5. **`target_plane` 在 ACQUIRING 阶段不算**,因为角点还没收敛;在 SEARCHING / LOST 立即 Invalidate。
6. **重新提交 RKNN 的判定**:`!track.active` 强制重提;`track.predicted_only` 说明最近没新测量也要重提;否则按 `frames_until_detection` 倒数。
7. **`mjpeg` 只有在有 client 连入才编码**,头上挂 `--mjpeg` 但没人看时**零开销**。

### 5.4 `DrawDebug` 的标记语

`DrawDebug` 是后人接手项目时最重要的"读图指南":

| 标记 | 颜色 | 含义 |
|---|---|---|
| 十字(灰) | (80,80,80) | 图像中心 |
| 矩形框 | 橙(0,180,255) | 当前跟踪 box |
| 四边形 | 绿(0,255,0) | refiner 角点(EMA 后) |
| 红色十字 | (0,0,255) | 跟踪器状态中心 |
| 倾斜十字 | 品红(255,0,255) | lead_time 外推的"瞄准点" |
| 黄色十字 | 蓝黄(255,255,0) | 单应矩阵反推的靶心 |
| 文字 | 行1: 状态+来源+conf+age; 行2: 帧率/NPU 容量 | 综合信息 |

`mjpeg` 流推的就是这张 overlay。

---

## 6. 相机采集 `CameraSource`(OpenCV + V4L2 异步)

`camera_source.cpp` 的设计是 **生产者-消费者**:`CaptureLoop` 线程死循环 `cap.read()`;主循环用 `WaitLatest(seq, ...)` 阻塞到"seq 推进了"。这样**主循环永远只看到最新帧,不积压**。

### 6.1 几个坑

1. `CAP_V4L2` 是默认 backend;如果打开失败就再试 `CAP_ANY`,这是为了在不同 libopencv 版本上都有回退。
2. `CAP_PROP_BUFFERSIZE=1` 必须设:V4L2 driver 默认会预拉若干帧,启动时观测到 ~3 帧延迟,设 1 后延迟固定为 1 帧。
3. `cap.set(cv::CAP_PROP_FOURCC, ...)` 必须放在 width/height/fps 之前,V4L2 协商顺序敏感。
4. `WaitLatest` 关键是 `if (seq_ != after_seq)`,所以**第一帧** `after_seq=0` 一定要能拿到。否则会把第一帧丢掉。

### 6.2 `cap.read` 失败的退避

```cpp
if (!cap.read(frame) || frame.empty()) {
  std::this_thread::sleep_for(2ms);   // 不是 busy 等
  continue;
}
```

读失败时睡 2 ms,**不是** `1ms busy poll`,这是为了给 USB/MIPI driver 时间恢复,但又不会让人眼看见延迟。

### 6.3 与 RKNN 时间的接口

每帧在锁内更新 `latest_ + timestamp_ + ++seq_`,然后 `notify_all()`。`timestamp_` 是 `cap.read()` **返回后**立刻打的时间,因此它是"`ts` 这幅图至少已经被采样/解压/搬到用户态"的最早时刻。配合 `predict_shrink_rate` 和 Kalman 投影,这个时间口径是够用的(120 Hz 升级时再切 native V4L2 buffer timestamp)。

---

## 7. NPU 推理 `RknnDetector`(全文最复杂的一个)

`rknn_detector.cpp` 1000 行是模型加载 / 推理 / 解码 / NMS / worker 调度的合集。下面按职责拆。

### 7.1 模型加载与能力查询

`LoadModel` / `QueryTensors`(第 7.4 节填好 `input_width_/height_/channels_/input_is_nchw_/output_attrs_`)。

- **自动覆盖**:如果模型实际 shape 与 `--input-size` 不同,会 `cout << "RKNN model input is XxY, overriding requested ..."`,运行时直接用模型读出来的。
- 输出 tensor:每个都缓存一份 `rknn_tensor_attr`,里面写了 `type / fmt / dims / zp / scale`,解码时按 type 分支(FP32 直拷 / FP16 转换 / INT8/UINT8 用 zp+scale 反量化)。

### 7.2 Zero-copy vs Legacy 输入

```cpp
if (config_.rknn_input_mode == "legacy") { /* legacy */ return; }
if (!SetupZeroCopyInput(&err)) {
  if (mode=="zero-copy") return false;
  cerr << "fall back to legacy";
}
```

`SetupZeroCopyInput` 做的事:

1. `RKNN_QUERY_NATIVE_INPUT_ATTR` 拿到原生输入 type/fmt。**必须** `FLOAT16` + `expected_fmt`(`NCHW` 或 `NHWC`),否则弃。
2. `rknn_create_mem(ctx, capacity)` 申请 DMA 内存,`capacity = max(native_bytes, attr.size, attr.size_with_stride)`。
3. `rknn_set_io_mem(ctx, mem, &attr)` 把这块内存注册成 input tensor。`attr.pass_through = 1` 表示"不要再做任何量化"。

跑一次推理时,zero-copy 路径:

```cpp
auto* native = (uint16_t*)input_mem_->virt_addr;
for (i=0; i<input_bytes; ++i) native[i] = LUT[input_data[i]];     // uint8 → fp16 LUT
rknn_mem_sync(ctx, mem, RKNN_MEMORY_SYNC_TO_DEVICE);
```

`NormalizedFp16Lut()` 是一个**懒构造的 256 元素查找表**(pre-norm 已经在 `Preprocess` 里除过 255,所以这里 LUT 是裸 `FloatToHalf(i/255)`)。如果每次推理重新算 256 个 fp16,在 ~73 Hz 下每秒浪费 1.8 万次转换。

> 实测 2026-07-12:**FP16 zero-copy 测得 inputs_set≈1.1 ms**;legacy `RKNN_TENSOR_UINT8` 同样字节数下要 ~10 ms。**这是项目里最容易看到的性能开关**。详见第 19.4 节。

### 7.3 8 段时间测量

`RknnDetector::Infer` 把一次推理拆成 8 个时间戳,塞进 `InferTimings`:

| 字段 | 含义 |
|---|---|
| `input_bytes` | 实际字节数 |
| `input_checksum` | `Fnv1a64(input_data, size)`,默认不计算 |
| `preprocess_ms` | `Preprocess()` 耗时(RGA 或 OpenCV) |
| `inputs_set_ms` | `rknn_inputs_set` / `mem_sync` 耗时 |
| `rknn_ms` | `rknn_run` 耗时 |
| `outputs_get_ms` | `rknn_outputs_get` 耗时 |
| `postprocess_ms` | `ToMatrix` + `DecodeOutputs` 耗时 |
| `select_ms` | `ChooseDetection` 耗时 |
| `outputs_release_ms` | `rknn_outputs_release` 耗时 |
| `total_ms` | 整段 |

为什么是这 8 个?`stats` 输出最初只打 pre/rknn/post/select,加起来不对,少 10 ms —— 加了 set/get/release 后才发现 `rknn_inputs_set` 内部有固定 ~10 ms 开销。这是 2026-07-12 报告第 5.6 节的"缺失的 10 ms"故事。

### 7.4 DecodeOutputs:YOLO 风格解码 + NMS

1. **合并多个输出分支**:模型通常有 3 个 head(8x/16x/32x stride),`DecodeOutputs` 把 row 数加起来到 `total_rows`,但 `width` 必须一致。
2. **归一化坐标启发式**:如果 `max(|cx|,|cy|,|bw|,|bh|) <= 2.0`,视作归一化(0~1 之间)→ 乘以 `input_w/h`。否则按模型输出的像素坐标处理。这是**对早期模型错配的兜底**,作者当年不想在配置里再加一个 bool。
3. **class 两种口径**:
   - `class_count == 1` → 直接 `item[4]` 当 score;
   - `class_count > 1` 且 `rest == class_count` → 没有 objectness,直接 argmax;
   - `class_count > 1` 且 `rest >= class_count+1` → `objectness × class_score`。
4. **map 回原图**:`(cx ± bw/2 - pad_x) / scale` 直接 Clamp 到画面。
5. **NMS**:conf 阈值过滤 → 按 conf 降序 → greedy IoU 抑制 → 累计到 `max_det`。

> 这段是项目"算法正确性"的命门 —— 2026-07-12 报告里 INT8 score 永远是 0、FP16 conf 0.87~0.94,差距就在这里被读出/被过滤掉。

### 7.5 ChooseDetection:多目标场景下的跟踪辅助

当 `prior` 有值(说明在跟踪中),不再简单取 conf 最大的,而是按 **0.35 × 距离分 + 0.30 × IoU 分** 加到 conf 上挑选。这是**对 ROI refiner 退化或 RKNN 多目标干扰的兜底**,确保跟踪不会突然被另一块很像的噪声骗走。

### 7.6 DetectorWorker:深度为 1 的"最新帧覆盖"队列

```cpp
void Run() {
  while (true) {
    unique_lock lk(mutex_);
    condition_.wait(lk, [this]{ return stop_ || pending_; });
    pending_ = false; busy_ = true; packet = move(pending_packet_); prior = move(pending_prior_);
    UnlockByScopeExit
    auto det = detector_->Infer(packet.frame, prior, &timings, &err);
    stat updates
    lock(mutex_); latest_result_ = ...; sequence_++; busy_ = false;
  }
}
```

设计要点:

- **latest-frame mailbox**:主循环每帧都调 `Submit`,若 worker 还在推理则**覆盖** `pending_packet_`,统计 `overwritten++`。这保证 NPU 永远在跑"最新"的图,而不是被旧帧拖累。详见 120Hz 设计 doc 第 7.2 节。
- **`smoothed_infer_fps`**:`InfERtotal` 的指数滑动平均(α=0.15)。**这是"处理容量 FPS"**,不是"实际完成检测频率"。
- **`smoothed_actual_fps`**:每 0.25 s 窗口统计"完成次数 / 窗口秒数",再用 α=0.5 平滑。**这是"实际完成 FPS"**,包含调度间隔。
- 两个 FPS 都报,目的是诊断:**如果 infer_fps ≫ actual_fps,说明主循环丢帧或 NPU 没吃饱**。
- `Stats()` 返回 `WorkerStats{submitted, overwritten, started, completed}`,`stats` 输出会把这些数量打出来。

### 7.7 fp16 LUT 实现技巧

`FloatToHalf` 是一个完整 IEEE 754 半精度转换(`std::memcpy` 看待 `float` 比特位)。`NormalizedFp16Lut()` 用 IIFE 静态初始化:`static const auto lut = []{...}();`,只在第一次调用时建表,后续 256 次查表完事。

---

## 8. RGA 硬件加速 `RgaPreprocessor`

板上 `/dev/rga` + `librga.so` 提供 RGA2 硬件 resize/imrotate/color convert。本类只做一件事:**把 BGR 1280×720 → 512×512 letterboxed RGB**,一步硬件完成。

### 8.1 接口

```cpp
bool Init(input_w, input_h, &err);   // 初始化 + 灰底 letterbox_buffer
bool Process(bgr_frame, &rgb_out, &scale, &pad_x, &pad_y, &err);
bool IsAvailable() const;
```

### 8.2 失败回退

`RknnDetector::Preprocess` 中先试 RGA:

```cpp
if (rga_preprocessor_.IsAvailable() && rga.Process(...)) { /* 用 RGA 输出 */ }
else { /* 走 OpenCV: cv::resize + cv::cvtColor(BGR2RGB) 填 letterbox */ }
```

letterbox 灰底 `(114,114,114)` 是 ImageNet 训练时常用的 mean-color 回填值。

### 8.3 性能

| 路径 | 耗时 |
|---|---|
| OpenCV CPU | ~12 ms |
| RGA 硬件 | ~2.2-2.5 ms |

> 当前**摄像头硬件上限 ~27.5 FPS 限制了 frame_fps**,RGA 不会让帧率上升,但能让一次推理 `total_ms` 从 ~33-42 ms 降到 ~23 ms,**给将来换 120 FPS 摄像头预留性能预算**。

### 8.4 im2d 失败兜底

`imresize_t` 失败时降级用 `imcvtcolor` 只做格式转换;再失败则 `RknnDetector::Preprocess` 整体走 OpenCV 路径。**任何一步失败都不会让进程退出**,对板端运行很关键。

---

## 9. ROI 矩形精修 `RectRefiner`

拿到 RKNN 给的 box,在它周围 ROI 内做经典视觉流程:**灰度 → 自适应阈值 → 形态学 → findContours → approxPolyDP → 评分 → cornerSubPix**。

### 9.1 关键动作

- **运动自适应 ROI**:`hints.speed` 越大,ROI 越大(`roi_expand_ratio × (1 + motion × motion_roi_gain)`),**避免快速目标飞出 box**。
- **运动自适应滤波**:运动 >0.5 时,`GaussianBlur` 用 3x3(否则 5x5),**避免过度侵蚀运动模糊的边缘**。
- **两级搜索**:threshold + morphology 没找到 → fallback 到 Canny + dilate 边缘图再搜一次;再次失败返回 nullopt。
- **四边形评分**:
  ```text
  score = (1 - iw) * (area 0.40 + extent 0.25 + center 0.13 + aspect 0.12 + right 0.10)
        + iw * iou_score   (iw = prior_iou_weight, 默认 0.20)
  ```
  这是个**加权因子**,把"面积占比"、"紧密性"、"中心接近参考"、"宽高比"、"直角程度"和"与上一帧 IoU"全部糅合。
- **直角约束**:`RightAngleScore` 算 `1 - |cos θ|` 对四个角取 min,<0.3 直接丢弃。
- **角点顺序**:`OrderCorners` 用 `(x+y)`/min、max + `(y-x)/min`、max 切出 TL/TR/BR/BL。
- **亚像素精修**:`cornerSubPix(5x5, EPS+MAX_ITER=30)`。

### 9.2 关键 bug 历史

2026-07-12 报告修了一个**多年漏网**的 bug:`findContours` 在 ROI 局部坐标,但 `RefinedRect.corners` 没加 `roi.x/y` 偏移,导致后续单应矩阵用错坐标。**单元测试 `TestRefinerCornersAreFrameCoordinates` 已经把这个回归焊死。**

### 9.3 评分门槛:宁缺毋滥

`refine_min_score=0.45`:评分低于此直接返回 `std::nullopt`,回退到裸 RKNN box。这是个**安全阀**,**绝不让劣质四边形覆盖掉 NPU 的好结果**。

---

## 10. 跟踪器 `RectTracker`(6 维 Kalman + 状态机)

跟踪器是项目算法含量最高、最有意思的文件。状态机:`SEARCHING → ACQUIRING → TRACKING → PREDICT_ONLY → LOST`。

### 10.1 6 维状态

```
x = [cx, cy, vx, vy, log_width, log_height]
z = [cx, cy, log_width, log_height]
```

中心是常速度模型,尺寸是 log-space 保证正数。测量矩阵 H 是 4×6 单位阵在 0/1/4/5 列上,这与 `statePost` 结构对应。

### 10.2 时间变形

`ConfigureTransition(dt)` 每帧根据实际 dt 设置 `transitionMatrix[0,2]=dt`、`transitionMatrix[1,3]=dt`,并**重新算过程噪声协方差**:

```
Q_pos = 0.25*dt^4*sigma^2
Q_cross = 0.5*dt^3*sigma^2
Q_vel = dt^2*sigma^2
```

其中 `sigma = max(1, kalman_process_noise)`,默认 600 px/s²。尺寸用 `0.25*size_sigma*dt` 一阶。

### 10.3 创新门控(Mahalanobis)

`PassInnovationGate` 算 `(z - H*x)` 与 Cholesky 解的协方差,得到马氏距离 `distance`。**默认阈值 13.28(= 卡方 4 自由度 99%)**。离谱的 RKNN 输出不会污染滤波器。

### 10.4 测量噪声按来源分

`ConfigureMeasurementNoise(source)`:

- ROI 测量:`center_sigma = kalman_roi_noise=2 px`,size 用基础 `kalman_size_noise=0.04`,乘 1.0。
- RKNN 测量:`center_sigma = kalman_rknn_noise=8 px`,size 乘 2.0。
- **运动模糊降权**:`if (speed >= motion_speed_ref)` 把 size_sigma 再放大 `size_motion_noise_gain`(默认 4x),**故意让"运动中检测框被拉大"那部分失真不破坏跟踪的尺寸记忆**。

这是 2026-07-12 报告里没讲但代码里有的关键优化。

### 10.5 预测期收缩 `predict_shrink_rate`

`Predict()` 在 misses≥2 时,给 `statePost.log_w / log_h` 减去 `rate × dt`,**让外推 box 主动收缩**,避免"运动模糊导致的大 box"在外推后还继续放大。这是**用先验几何知识补偿 RKNN 的偏置**。

### 10.6 corners 变换

`TransformCorners(old, new)` 把先前 4 角按 `(new - old) / old` 做平移 + 等比缩放。**不独立预测每个角**,因为那样会让四边形变形;只平移+缩放保证矩形性。

### 10.7 状态机规则(精简)

| 当前状态 | 接受测量 | 更新后状态 |
|---|---|---|
| 任何非 TRACKING | `roi`: 必须 `roi` + `IsFresh`;否则拒绝 | 视情况 |
| SEARCHING/LOST | 仅锚测量(initialize) | ACQUIRING |
| ACQUIRING 连击 ≥ 2 | 任何 | TRACKING |
| ACQUIRING 超时 | ROI 失败 | LOST |
| TRACKING | 任何(过门控) | TRACKING |
| 任意当前状态 | 无新测量 | PREDICT_ONLY |

### 10.8 lead-time 外推

`Snapshot().predict_center = (cx + vx*lead_time, cy + vy*lead_time)`,`lead_time=0.05 s`。**这是给云台"50 ms 后"瞄准用的点**,刚好补偿 STM32 控制环和机械延迟。

### 10.9 UpdateCorners 的 EMA

```cpp
if (max_jump > corner_reset_dist) corners_ = raw;     // 大跳直接覆盖
else corners_[i] = corners_[i] * (1 - alpha) + raw[i] * alpha;  // alpha=0.5 平滑
```

防止某次 ROI 抽风让四边形抖掉。

---

## 11. 靶面几何 `TargetPlane`(单应矩阵 + 厘米坐标)

`target_plane.cpp` 实现"已知图像 4 角 → A4 物理 4 角"的双向单应矩阵,并验证凸性、面积、边长、纵横比。

### 11.1 四角规范化

`OrderCorners` 以 4 中心为参照做极角排序(逆时针),再用 `SignedArea2` 校正方向,**保证永远 BL→BR→TR→TL**(与相机 y 轴向下兼容)。

### 11.2 纵横比横竖自动判断

观测纵横比接近 `W/H`(竖版)还是 `H/W`(横版),取 log-差小者。**首次 TRACKING 后锁定朝向**,避免相机接近矩形死角时横竖来回抖。

### 11.3 单应矩阵

```cpp
image_to_target = getPerspectiveTransform(image_pts, target_pts_cm);
target_to_image = getPerspectiveTransform(target_pts_cm, image_pts);
```

存储两边矩阵可以避免每次反推。

### 11.4 验证项(全部过才算)

- 四角凸性(相邻边叉积符号一致)。
- 面积 ≥ `max(16 px², image_area × 1e-4)`。
- 最短边 ≥ `max(2 px, diag × 0.002)`。
- 纵横比误差 [0.35, 2.85](对应 ~3x 公差)。
- 矩阵元素全部 finite。
- 行列式绝对值 ≥ 1e-12(非退化)。
- 4 个**回投误差 ≤ 2 px**(自洽误差)。

### 11.5 调用门槛

`main.cpp` 里只在 `track.active && state != "ACQUIRING" && has_corners` 时调用 `Update`。其它情况 `Invalidate()`,**任何不满足的状态都立即 `valid_=false`**。这是**防止跟踪器短时外推导致单应矩阵"假阳性"地漂移**。

### 11.6 厘米坐标对外接口

```cpp
std::optional<Point2f> TargetToImage(Point2f cm);  // 圆周点 → 图像
std::optional<Point2f> ImageToTargetCm(Point2f px);  // 图像 → 厘米
```

后者在 AimWriter / 后续控制器里不使用(没有反推需求),保留作将来调试和云台坐标变换用。

---

## 12. 输出协议 `AimWriter`(ASCII + 12 字节二进制)

### 12.1 ASCII `$AIM` 行(stdout)

```
$AIM,found,state,cx,cy,off_x,off_y,norm_x,norm_y,
     px,py,pnorm_x,pnorm_y,vx,vy,conf,source
```

- `found=1` 时填有效值,否则全 0。
- 偏移 / 归一化同时给出,下游选其一。
- `state` ∈ `SEARCHING / ACQUIRING / TRACKING / PREDICT_ONLY / LOST`。
- `source` ∈ `none / rknn / roi / roi+rknn / predict`。

用 `snprintf(buffer, 320, ...)` 一行写完,**没有 buffer 拼接**,简单粗暴。

### 12.2 12 字节二进制串口协议(给 STM32 erweiyuntai)

```
AA 08 xHi xLo yHi yLo vxHi vxLo vyHi vyLo checksum 55
```

- 起始 `AA`,长度 `08`(固定负载 8 字节),结束 `55`。
- `checksum = sum(bytes[1..9]) & 0xFF`,**不加密只求和**,STM32F1 端单周期加法足够。
- `x/y` 通过 `scale = packet_frame_width / capture_width` 等比例缩到 STM32 的"接收帧坐标"(默认 800×480)。
- **clamp 到 [1, w-2] 和 [1, h-2]** —— STM32 把 0 当"无效",把 ≥w/h 当"out of bounds 拒绝",所以视觉侧的合法值不会撞这两类拒绝。
- `vx/vy` 用 `int16_t`(±32767),允许负数表示反向;也是按同样比例缩放。

### 12.3 节流

`serial_packet=true` + `serial_interval=0.02` → 50 Hz 串口包。`serial_interval` 是按"上次发送时刻"判断,而不是按帧计数,保证无论摄像头帧率如何都不发超出 STM32 处理能力的包。

stdout 节流相反:**镜头画面事件**出现才打,所以是 `--stdout-interval=0.2` → 5 Hz。

---

## 13. 远程 MJPEG `MjpegServer`

板子无头,ssh 远程调试时 `cv::imshow` 看不见,所以单独起一个轻量级 multipart/x-mixed-replace HTTP 服务把带标注 overlay 推给浏览器。

### 13.1 重要设计

1. **没人看零开销**:`PushFrame` 第一行 `if (client_count_ == 0) return;`,没有 JPEG 编码,没有 mutex;后面的视觉主循环**永远不被 MJPEG 拖慢**。
2. **latest-frame wins**:服务端只缓存最新一帧,JPEG buffer 用 `swap` 立即替换;慢客户端直接"跳帧",不会让主循环等网络。
3. **accept 不阻塞 Stop**:`Stop()` 先 `shutdown(SHUT_RDWR)` 再 `close`,确保 accept 立刻返回 -1,然后 join。
4. **MSG_NOSIGNALE**:对端关闭时 `send` 返回 EPIPE 不杀进程。
5. **TCP_NODELAY**:小包低延迟,JPEG 每帧都要及时送。

### 13.2 浏览器兼容

`multipart/x-mixed-replace` 是 90 年代的老格式,**所有现代浏览器**(Chrome / Firefox / Safari / Edge)依然原生支持直接展示。无需 JavaScript,无需 HTML,直接打开 `http://<board-ip>:8080/` 就是视频流。

---

## 14. 线程亲和性 `thread_affinity`(嵌入式性能关键)

RK3576 是 4 小核 (0-3) + 4 大核 (4-7) 的 big.LITTLE。视觉主循环 + 摄像头采集 + NPU 推理 worker 都是**忙等**:

- 主循环等帧(100 ms `wait_for`)但等待结束立即密集处理,不睡。
- 摄像头采集线程 `cap.read()` 内阻塞 + 2 ms 间隔。
- NPU worker 锁等 condition 后立刻 13 ms 推理。

三个忙等线程如果不绑核,OS 调度器会随机把它们都排在小核,**主循环采集速率会出现 13.7/27.5 偶发减半**(2026-07-12 报告第 5.2 节)。绑到 5/6/7(大核 + 错开)后稳定 27.5 FPS。

`BindCurrentThread(spec)` 支持的语法:
- `"5"`: 单核。
- `"4-7"`: 范围。
- `"0,2,4,6"`: 逗号分隔任意组合。
- **空字符串 = 不绑**(返回 true)。

> 这是"无需 root 即可大幅提升性能"的典型例子。

---

## 15. 命令行 / 配置 `config.cpp` 全参数手册

`ParseCommandLine` 解析约 70 个参数。下面按"调一次就改 vs 项目固化"分类。

### 15.1 系统级(固化,几乎不该改)

| 参数 | 默认 | 说明 |
|---|---|---|
| `--model` | `../../models/best_n_stage1_512_fp16.rknn` | 模型路径 |
| `--camera` | `/dev/video73` | 摄像头设备 |
| `--width/--height/--fps/--fourcc` | 1280/720/30/MJPG | 协商参数 |
| `--core-mask` | `01` | NPU 双核;`default` 不绑 |
| `--rknn-input-mode` | `auto` | `auto` / `legacy` / `zero-copy` |
| `--camera-affinity/--detector-affinity/--main-affinity` | 空 | 见第 14 节 |

### 15.2 算法阈值(项目固化)

| 参数 | 默认 | 含义 |
|---|---|---|
| `--conf` | 0.25 | NMS 前分数下限 |
| `--iou` | 0.50 | NMS IoU 阈值 |
| `--max-det` | 1 | 只关心 1 个目标 |
| `--min-box-size` | 6.0 px | 太小的噪声框丢弃 |
| `--refine-min-score` | 0.45 | 弱四边形不要 |
| `--target-width-cm/--height-cm` | 21.0 / 29.7 | A4 横版;比赛前必须按实测值改 |
| `--class-count / --target-class` | 1 / 0 | 单类 |
| `--kalman-process-noise` | 600 | px/s² 加速度噪声,适合厘米级移动目标 |
| `--kalman-rknn-noise / -roi-noise` | 8 / 2 | 双源位置噪声 |
| `--kalman-size-noise` | 0.04 | 对数尺寸噪声 |
| `--kalman-gate` | 13.28 | 卡方 4 DoF 99% |
| `--size-motion-noise-gain` | 4.0 | 高速时 size 不被外推污染 |
| `--predict-shrink-rate` | 1.2 | 连续 miss 时 box 主动收缩 |
| `--lead-time` | 0.05 s | 云台瞄准前瞻 |

### 15.3 调试/可选

| 参数 | 说明 |
|---|---|
| `--window / --no-window` | 本地 cv::imshow(SSH 不可见) |
| `--mjpeg-port N` | 启用远程 MJPEG,默认 0=关 |
| `--mjpeg-quality` | 70 |
| `--serial-port` | 启用串口下发 |
| `--serial-packet / --serial-ascii` | 12 字节 vs `$AIM` |
| `--packet-frame-width/height` | STM32 接收端坐标(默认 800×480) |
| `--baudrate` | 115200 |
| `--serial-interval` | 0.02 s = 50 Hz |
| `--stdout-interval` | 0.20 s = 5 Hz |
| `--stats-interval` | 1.0 s |

任何**合法性检查**在 `ParseCommandLine` 末尾:kalman / size / target 等必须 > 0,不合法直接 return false。

---

## 16. 时间戳、帧号、延迟度量契约

整个流水线的"时间观"是项目的灵魂,不读懂这节就读不懂 120Hz 升级设计。

### 16.1 四个时间点

```
t_capture_done   = cap.read() 返回后立刻打的稳态时间        // 摄像头线程
t_infer_start    = worker 开始调 detector.Infer()           // detector 线程
t_infer_done     = detector.Infer() 返回                   // detector 线程
t_publish        = main loop 末尾 writer.Send() 前打         // main 线程
```

加上 tracker 内部:
```
state_ts              = 当前滤波器状态的时间戳
last_measurement_ts   = 上次被 correct() 接受的时间戳
query_frame_ts        = 触发本轮 main loop 的 capture_done_ts
```

### 16.2 不变量(全部由单元测试 + 主循环 enforce)

- 所有时间戳用 `steady_clock`,全程单调,只要在进程内不重启就相对原点可比。
- `tracker.Predict` 在 `main` 每帧**强制**调一次,把滤波器时间推到 `query_frame_ts`。
- worker 结果的 `source_capture_ts` 记录的是**当时被推理那一帧**的 capture_done_ts,不会改成 `t_infer_done`,杜绝"用推理结束时间冒充测量时间"的常见错误。
- `tracker` 只接受 `measurement_ts ≤ state_ts + ε` 的测量,且 `state_ts` 永不退。
- `PredictTo` 用 dt = min(真实 dt, prediction_timeout) 防大跳。

### 16.3 延迟度量口径

`stats` 输出这两条:

- `result_age_ms = (query_capture_ts - result.source_capture_ts) * 1000` —— 这次用的 RKNN 结果**几毫秒前**算的。
- `measurement_age_ms = (publish_ts - last_measurement_ts) * 1000` —— Tracker 上次真正接受的测量**几毫秒前**。

这两个指标分开,是为了排查"RKNN 慢"和"测量太旧"两件事。

---

## 17. 性能优化的 6 条路线(全部已落地)

按 2026-07-12 实测报告整理。所有数字都是板上 1280×720 MJPG 跑出来的。

| # | 优化项 | 改动 | 效果 | 备注 |
|---|---|---|---|---|
| 1 | 模型从 INT8 改 FP16 | `config.hpp` 默认;只删 INT8 路径 | conf 0.0 → 0.87+ | 模型本身就坏 |
| 2 | RKNN 双核 | `--core-mask 01` | rknn 19.7ms → 16ms(-19%) | 硬件能力 |
| 3 | FP16 zero-copy | `--rknn-input-mode auto`;`SetupZeroCopyInput` 套上 | inputs_set 10ms → 1.1ms(-89%) | 本仓库最大单点优化 |
| 4 | RGA 硬件预处理 | `RKFAST_USE_RGA=ON` | pre 12ms → 2.4ms(-80%) | 不影响 frame_fps 但降 total |
| 5 | 线程亲和性 | `--camera-affinity 5 --detector-affinity 6 --main-affinity 7` | frame_fps 13.7→27.5(消除跳变) | 完全消除双峰 |
| 6 | CPU/NPU/DDR performance governor | `tools/rk3576_performance.sh` | rknn 16ms → 10ms;frame_fps 上限给满 | 默认每次跑前 sudo 执行 |

**所有 6 项都已写进 `run_camera.sh`,用户开箱即享受到**。

### 17.1 实测耗时链(RGA+双核+性能+zero-copy)

```
pre=2.4ms   set=10.0ms  rknn=10.1ms  get=0.1ms
post=0.2ms  select=0.0ms release=0.0ms total=22.9ms
```

`set ≈ rknn ≈ 10ms`:这两个是**这颗 NPU 当前的固定架构开销**,2.3.2 升级前后一样。详见 2026-07-12 报告第 7 节。

---

## 18. 线程模型

```
进程: rkfast_target
├─ thread: main            绑 main_affinity    ; 主循环 + 上层决策
├─ thread: camera          绑 camera_affinity  ; cap.read() 持续采
├─ thread: detector-worker 绑 detector_affinity; RKNN 后台推理(mailbox)
└─ thread: mjpeg-acceptor                           ; 仅当 mjpeg 启用
   ├─ client-1
   ├─ client-2
   └─ ...
```

### 18.1 同步点

- `camera → main`:条件变量 + seq 比较(`WaitLatest`)。
- `main → detector`:互斥锁 + 条件变量(`Submit`)。
- `detector → main`:互斥锁保护 `latest_result_`(`LatestResult`)。
- `mjpeg → 主线程`:mjpeg 有自己的 frame_mutex_,但只编码,不阻塞 main。

### 18.2 锁设计原则

- `camera.latest_` 持有时间仅一次 cv::Mat 移动 + timestamp 写 + seq++,**绝不做图像处理**。
- `detector.mutex_` 持有时间很短,大量时间在 unlock 状态下推理。
- `mjpeg.frame_mutex_` 仅保护 jpeg_buffer swap 与 seq++,**不在锁内做 send**。

### 18.3 不需要 thread pool 的原因

板端总共就这么几个"长期"线程,每个职责清晰,加 pool 反而把延迟预算打散。

---

## 19. 工具集

### 19.1 `image_probe`(静态图诊断)

不开摄像头、不接串口,用**生产环境的 `RknnDetector` + `RectRefiner` 代码路径**跑一张磁盘上的图。`cpp_fast_targeting/tools/image_probe.cpp` + 一组共用 `.cpp`。

用途:模型有更新时,**不需要摄像头就能在板上(或 dev 主机)跑回归**。

### 19.2 `input_mode_compare`(三种 input mode 性能对比)

`auto / legacy / zero-copy` 三种模式逐一跑,输出 `pre / set / rknn / get / post / select / release` 7 段时间。**这是 2026-07-12 报告第 5.6 节那份"9.9 ms 固定开销"数据的原始来源**。

### 19.3 `tools/rk3576_performance.sh`

把 CPU policy0+4 governor 改 performance;NPU / DDR governor 改 performance。**不写 scaling_setspeed**,避免与板载 thermal 管理对抗。

### 19.4 `tools/rk3576_monitor.sh` / `tools/test_rga.sh`

板上状态观测和 RGA 设备裸测,辅助排错。

---

## 20. 测试 `rkfast_tests.cpp`

子项目 `cpp_fast_targeting/tests/CMakeLists.txt` 编译出 `rkfast_tests`,仅覆盖**不依赖 NPU/RGA/摄像头**的纯算法逻辑:

- `TestBoxIouBasic / TestBoxIouEmpty / TestBoxIouDegenerate`
- `TestBoxClipImage`
- `TestTargetPlaneCornersOrderUnordered`
- `TestTargetPlaneAspectLocked`
- **`TestRefinerCornersAreFrameCoordinates`**  ← 2026-07-12 修复的关键回归
- `TestRefinerMotionExpansion`
- `TestRectTrackerConstantVelocity`
- `TestRectTrackerMissingFrames / TestRectTrackerReacquire`
- `TestTrackerConfidenceTimeout`
- `TestAimWriterBinaryPacketChecksum`
- `TestAimWriterCenterVelocityScaleAndClamp`

> **设计原则**:**任何能用 C++ unit test 验证的都不上摄像头**;RKNN / RGA / Camera / Serial 全部交给 `run_camera.sh` 实际跑验证。

---

## 21. 板端运行步骤(给接手的人)

### 21.1 一次性安装

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev
# librknnrt 和 librga 由板卡 SDK 或 Rockchip 官方源提供
sudo ldconfig
```

### 21.2 构建并自检

```bash
cd ~/rk3576_rknn_targeting/cpp_fast_targeting
bash build_and_verify.sh    # 10 步验证
```

### 21.3 启动

```bash
bash run_camera.sh                    # 默认:FP16 zero-copy + RGA + 双核 + 性能模式
bash run_camera.sh --serial /dev/ttyS4   # 加串口下发(STM32 erweiyuntai)
bash run_camera.sh --mjpeg 8080          # 远程预览
bash run_camera.sh --no-perf             # 排查时跳过性能模式
bash run_camera.sh --input-mode legacy   # 切到 legacy 输入路径
```

### 21.4 远程看画面

浏览器打开 `http://<board-ip>:8080/`,看到的是带检测框/角点/外推点的 overlay。

---

## 22. 性能现状与 120Hz 升级路线

### 22.1 当下瓶颈结构

| 瓶颈 | 上限 |
|---|---|
| 摄像头硬件 `/dev/video73` | ~27.5 FPS(任何分辨率 ≤ 720p) |
| NPU `set + rknn` | ~20 ms / 次 → 50 FPS 上限 |
| NPU `set` 固定开销 | ~10 ms(NPU 驱动架构级别,无法纯靠升级 `rknn-toolkit2` 改善) |

> 当前总帧率受限于摄像头硬件。**RGA 等优化项不影响 frame_fps,只为换 120Hz 摄像头预留预算**。

### 22.2 120Hz 升级的 9 步(摘自设计文档)

1. 时间戳 + FramePacket + DetectionResult 元数据(已就绪,`InferResult` 已有 `source_capture_ts/seq`)。
2. CameraSource 等帧接口 + DetectorWorker latest-frame mailbox(已就绪,见 7.6 节)。
3. VisualTracker + 主循环接入(已就绪)。
4. 状态机、时间失效规则、门控(已就绪,见 10.7)。
5. TargetPlane 几何 + 单应(已就绪)。
6. 单元测试 + 调度 + 几何测试(已就绪,见 20 节)。
7. `vision_replay` 与 120 FPS 数据回放(待写)。
8. 当前 30 FPS 回归(已通过,2026-07-12)。
9. **120 FPS 摄像头实测 + 参数整定(下一个里程碑)**。

### 22.3 不进首版的两件事

- **常加速度 Kalman**:90 度转弯时易沿旧方向过冲,首版用常速度 + ROI/NPU 多速率融合兜底。
- **LK 光流 + 激光光点**:实测数据未证明 ROI 合格率 < 95% 或 ROI p99 > 5 ms 之前不引入。

---

## 23. 关键设计权衡(给后人留的"为什么")

| 决策 | 替代方案 | 选择 |
|---|---|---|
| 单进程单生产者采集 | 双缓冲 + frame queue | 摄像头硬件采集速率 < NPU 速率,双缓冲没收益 |
| `latest-frame mailbox` | FIFO 多帧队列 | 旧帧做 NPU 推理是浪费;过期的检测还会骗 tracker |
| log-space size | 像素 space | size 必须正,exp() 出来的尺寸始终 > 0;log 噪声也对称 |
| 6 维 Kalman | α-β 滤波 | α-β 不能按 source 调噪声,也不能门控异常值 |
| ROI refiner 评分兜底 | 全靠 RKNN | RKNN 的 box 是粗定位,角度/比例有偏,需要几何级精修 |
| target_plane 横竖锁定 | 实时切换 | 透视接近死角时会来回切,锁定防抖 |
| trk 时长失效(100 ms) | 无限 predict | 摄像头丢帧时无限外推会导致云台追到一个不存在的目标 |
| size_motion_noise_gain | 全程按 size_sigma 信任 RKNN | 运动模糊拉大 box 让 size 跟着漂,降权后稳定 |
| predict_shrink_rate | 不动 size | 连续 miss 时主动收缩,让下一帧 ROI 不至于包含太多背景 |
| 两个 FPS 指标 | 只报一个 | infer_capacity vs actual_fps 分离可区分 NPU 没吃饱 vs 调度丢帧 |
| `--conf 0.25` 这么低 | 用 0.5 / 0.7 | 2026-07-12 实测该模型最高 0.94,0.25 是漏检与误触发平衡点 |
| 12 字节二进制串口 | NMEA 风格 ASCII | STM32 端单轮加和校验,无字符串解析开销;且字段定长 |
| `MSG_NOSIGNALE` + `TCP_NODELAY` | 默认 | 客户端突然关连时进程不被 SIGPIPE 杀掉;MJPEG 流即时更新 |
| Capture 失败睡 2ms | busy poll | V4L2 设备暂时不可用时(USB 拔插)给 driver 留恢复时间 |

---

## 24. 待办 & 改进空间

按优先级:

| 优先级 | 项目 | 上下文 |
|---|---|---|
| P0 | 换 120 Hz 摄像头模组 | 解锁 18 节 6 项优化带来的实际帧率收益 |
| P0 | 等 Rockchip 发布 NPU 内核驱动 0.9.8+ | 消除 set 10ms 固定开销 |
| P1 | 上 LK 光流 | 实测 ROI 合格率 < 95% 或 p99 > 5 ms 时启动 |
| P1 | vision_replay 工具 | 离线回放 ROI/Tracker 调整 |
| P1 | 激光光点检测 | 405nm 激光在不同曝光下的先导验证 |
| P2 | 模型压缩(剪枝 / 蒸馏) | 当前 13.6ms 总耗时还有余量,可不压缩;真换 120Hz 就要做 |
| P2 | UVC 摄像头热插拔恢复 | 当前 sleep 2ms 兜底,可改成异步重连 |
| P3 | MPI/MSPM0 车辆侧协议 | 已规划,见 120Hz 设计第 4 节 |

---

## 25. 排错速查表

| 现象 | 看哪里 |
|---|---|
| 程序能跑但一直 `SEARCHING` | `stats conf=...`、是否 < 0.25,先 `image_probe` 跑训练图看 RKNN 直出分数 |
| `tracker confidence=0` 但 NPU 命中 | `predict_decay` 是否太大,或上一帧 target 飞出画面 |
| 帧率不稳 | 没绑核心亲和性 |
| MJPEG 流黑屏 | `client_count_=0` 一定是没连入;若非 0 看 JPEG 编码是否返回 false |
| 串口无输出 | `--serial-port` 没设,或 STM32 端检查 baud 与 checksum 算法 |
| `rknn_inputs_set` 仍然 > 5 ms | `--rknn-input-mode` 没切到 auto/zero-copy,或 native attr type 不是 FP16 |
| `RGA found: ...` 不出现 | RGA 设备不可用,看 `ldconfig -p \| grep librga` |
| refiner 角点全错位 | `refine_min_score` 是否被调到太低 |

---

## 26. 附录:全套默认参数速查表

```cpp
// 模型与硬件
model_path               = "../../models/best_n_stage1_512_fp16.rknn"
camera                   = "/dev/video73"
use_v4l2                 = true
width                    = 1280
height                   = 720
fps                      = 30
fourcc                   = "MJPG"
input_size               = 512
target_class             = 0
class_count              = 1
conf_threshold           = 0.25
iou_threshold            = 0.50
max_det                  = 1
min_box_size             = 6.0
warmup_runs              = 3
core_mask                = "01"          // NPU 双核
rknn_input_mode          = "auto"        // 优先 zero-copy FP16
verify_input_checksum    = false
detect_interval_locked   = 0
max_predict_misses       = 3
track_alpha              = 0.55
track_beta               = 0.20
track_box_alpha          = 0.45

// Kalman
kalman_process_noise     = 600.0
kalman_roi_noise         = 2.0
kalman_rknn_noise        = 8.0
kalman_size_noise        = 0.04
size_motion_noise_gain   = 4.0
predict_shrink_rate      = 1.2
kalman_gate_threshold    = 13.28
max_track_speed          = 4000.0
acquire_timeout          = 0.10
prediction_timeout       = 0.10
homography_timeout       = 0.05
confidence_decay_tau     = 0.15

// Velocity
lead_time                = 0.05
predict_decay            = 0.9

// Refiner
roi_expand_ratio         = 0.24
roi_min_size             = 24
refine_min_area_ratio    = 0.05
refine_min_score         = 0.45
corner_alpha             = 0.5
corner_reset_dist        = 40.0
prior_iou_weight         = 0.20
motion_roi_gain          = 1.5
motion_speed_ref         = 300.0

// A4 靶面
target_width_cm          = 21.0
target_height_cm         = 29.7

// 输出与调试
show_window              = false
mjpeg_port               = 0
mjpeg_quality            = 70
stats_interval           = 1.0
opencv_threads           = 1
serial_port              = ""            // 默认不启用串口
baudrate                 = 115200
stdout_interval          = 0.20          // 5 Hz
serial_interval          = 0.02          // 50 Hz

// STM32 协议(erweiyuntai)
serial_packet            = true
packet_frame_width       = 800
packet_frame_height      = 480
```

---

## 尾声

这份仓库不是"一个看起来能跑的项目",而是**一段进化史的当前快照**:从 `archive/python-runtime` 的 slin 高帧率设计,到 INT8 失败、FP16 切换、双核、zero-copy、RGA、线程亲和性、性能 governor,再到 2026-07-13 的 120 Hz 升级路线 —— 每一项落地动作都有数据支撑、有回归测试、有文档沉淀。

接手时强烈建议:
1. **先看 `docs/reports/RK3576_矩形识别修复与性能优化报告_2026-07-12.md`**,把基线数字固化到脑中。
2. **再读 `docs/superpowers/specs/2026-07-13-rk3576-120hz-vision-tracking-design.md`**,知道下一步该往哪走。
3. 然后**用本文做导航**,去每份 `.cpp` 里找具体实现。
4. 改任何算法参数都跑 `bash build_and_verify.sh` + `image_probe` 一次。

存疑时永远回到这个顺序:**测量第一 → 调参第二 → 重构第三**。
