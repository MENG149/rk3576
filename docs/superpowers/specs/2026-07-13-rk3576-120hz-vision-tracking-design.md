# RK3576 120 Hz 靶面视觉跟踪与补帧设计

日期：2026-07-13
状态：已批准设计，待实施计划
范围：`rk3576_rknn_targeting/cpp_fast_targeting` 的视觉采集、检测、ROI 精修、状态估计、靶面几何与观测指标。

## 1. 背景

本项目用于 2025 年全国大学生电子设计竞赛 E 题简易自行瞄准装置。发挥题要求小车沿 100 cm x 100 cm 方形轨迹运动时，激光持续照射固定靶面，并完成以下视觉相关目标：

1. 移动过程中持续定位靶心，为 `D1 <= 2 cm` 的系统指标提供稳定视觉基准。
2. 在靶面物理坐标中生成半径 6 cm 的圆周参考点，支持小车一圈对应光斑一圈。
3. 将未来 120 FPS 相机与当前 RKNN 推理链路组合，使用高频 ROI 测量和短时预测填补没有新 RKNN 结果的相机帧。

当前板端实测基线：

- 相机：1280 x 720 MJPG，约 27.5 FPS，现有相机标称上限 30 FPS。
- RKNN 单次完整链路：约 13.6 ms，对应计算容量约 73 FPS。
- 纯 `rknn_run`：约 9.7 ms。
- RGA 预处理：约 2.5 ms。
- FP16 输入转换与同步：约 1.1 ms。
- 当前 `infer_fps` 是单次处理耗时的倒数，不是单位时间内实际完成的检测数量。

在 120 FPS 输入下，帧周期为 8.33 ms。现有 worker 只在空闲时接受提交，因此完整 13.6 ms 推理会被相机帧节拍量化，实际检测约为 60 Hz。视觉主链路需要在每个相机帧上产生最新靶面估计，而不是等待每一帧都完成 RKNN。

## 2. 目标

本设计实现以下能力：

1. 为每个相机帧建立单调、可追踪的帧号和时间戳。
2. 将 RKNN worker 改为深度为 1 的 latest-frame mailbox，使其完成当前推理后立即处理最新待处理帧。
3. 使用 120 Hz ROI 四角精修作为主要测量，使用约 60 至 73 Hz RKNN 作为全图发现、重捕获和漂移重置。
4. 使用可变时间步长的常速度 Kalman 滤波器，预测中心、速度和框尺寸，填补短时无测量帧。
5. 从可靠四角建立图像与 A4 靶面厘米坐标之间的单应矩阵。
6. 输出带时间戳、测量年龄、来源和置信度的 `VisualEstimate`，供后续云台控制使用。
7. 建立可回放、可量化的性能和精度验证方法。

## 3. 非目标

本阶段不实现以下内容：

- 云台电机位置环、速度环和前馈控制。
- MSPM0 车辆里程、IMU 和圈进度融合。
- RK3576 到云台控制器的最终二进制串口协议。
- 现有 FP16 RKNN 模型重新训练、剪枝或量化。
- 默认启用 LK 光流。
- 默认启用 405 nm 激光光点检测。

接口会保留靶面厘米坐标、时间戳和置信度，使上述能力可以在后续阶段接入。

## 4. 方案比较与决策

### 4.1 继续使用现有 alpha-beta

优点是改动小、无新增 OpenCV 组件。缺点是当前固定增益无法根据测量来源和噪声调整，时间戳倒退会被钳成 1 ms，速度和失锁行为依赖帧率。

结论：不作为最终方案，只作为回归基线。

### 4.2 可变 dt 常速度 Kalman + ROI/NPU 多速率融合

优点是能显式维护协方差、按测量质量调整噪声、处理 8 至 17 ms 的短时空档，并与现有 ROI refiner 和 RKNN worker 边界匹配。

结论：采用。

### 4.3 常加速度 Kalman 或完整光流/单应性跟踪

常加速度图像模型在小车 90 度转弯时容易沿旧方向过冲。光流能提供高频图像测量，但会受运动模糊、弱纹理和错误特征影响，不能单独承担长期状态估计。

结论：常加速度模型不进入首版。LK 光流仅在实测证明 ROI 精修成功率或时延不满足要求时加入。

## 5. 总体架构

```text
CameraSource 120 Hz
  -> FramePacket(frame, seq, capture_done_ts)
  -> Main vision loop
       -> current-frame ROI quadrilateral refinement
       -> optional short-gap LK measurement
       -> VisualTracker predict/correct
       -> TargetPlane homography validation
       -> VisualEstimate 120 Hz

DetectorWorker latest-frame mailbox
  <- FramePacket + tracker prior
  -> RKNN DetectionResult
       source_seq
       source_capture_ts
       infer_start_ts
       infer_done_ts
       detection
  -> full-frame anchor, reacquire, drift reset
```

模块职责保持单一：

- `CameraSource`：采集、帧号、时间戳和实际相机参数回读。
- `DetectorWorker`：最新帧覆盖队列、RKNN 推理及源帧元数据传播。
- `RectRefiner`：在当前相机帧的 ROI 中输出四角、框和质量分数。
- `VisualTracker`：单调时间状态、Kalman 预测/校正、状态机和置信度。
- `TargetPlane`：四角排序、几何验证、单应矩阵和厘米坐标转换。
- `VisionStats`：采集、检测、精修、跟踪、几何和帧龄统计。

## 6. 时间戳与帧数据契约

### 6.1 FramePacket

```cpp
struct FramePacket {
  cv::Mat frame;
  uint64_t seq;
  double capture_done_ts;
};
```

`capture_done_ts` 表示当前 OpenCV/V4L2 路径中 `cap.read()` 成功返回后的单调时钟时间。它不是传感器曝光起点，命名必须反映这一事实。未来若切换原生 V4L2 buffer timestamp，可扩展为独立的 sensor timestamp。

### 6.2 DetectionResult

```cpp
struct DetectionResult {
  uint64_t result_seq;
  uint64_t source_frame_seq;
  double source_capture_ts;
  double infer_start_ts;
  double infer_done_ts;
  std::optional<Detection> detection;
  InferTimings timings;
};
```

禁止再用推理结束时间冒充检测框的测量时间。

### 6.3 VisualEstimate

```cpp
struct VisualEstimate {
  uint64_t query_frame_seq;
  double state_ts;
  uint64_t last_measurement_frame_seq;
  double last_measurement_ts;
  double publish_ts;
  double state_age_ms;
  double measurement_age_ms;
  TrackState state;
  EstimateSource source;
  Box box;
  std::array<cv::Point2f, 4> corners;
  bool has_valid_homography;
  cv::Point2f target_center_px;
  cv::Matx33f image_to_target;
  cv::Matx33f target_to_image;
  float vx_px_s;
  float vy_px_s;
  float confidence;
};
```

`query_frame_seq` 和 `state_ts` 标识本次估计所对应的当前相机帧；`last_measurement_frame_seq` 和 `last_measurement_ts` 标识最后一次真正被 Kalman 接受的 ROI 或 RKNN 测量。`publish_ts` 是发布时刻，`state_age_ms = publish_ts - state_ts`，`measurement_age_ms = publish_ts - last_measurement_ts`。预测帧不得把 query frame 伪装成新测量帧。后续控制器可据此区分计算延迟和真实测量陈旧程度。

## 7. 采集和调度

### 7.1 CameraSource

- 增加等待新帧接口，使用条件变量替代主循环空转读取相同帧。
- 每次成功采集后递增 `seq`，保存 `capture_done_ts`。
- 请求分辨率、FPS、FOURCC 和 buffer size 后，回读并打印实际协商值。
- 统计实际帧率、序号间隔、采集失败和主循环漏帧。
- 继续只保留最新帧，不建立历史图像队列。

### 7.2 DetectorWorker

- `Submit` 无论 worker 是否正在推理，都更新一个深度为 1 的 pending slot。
- 主循环在每个满足检测调度条件的新相机帧上直接调用 `Submit`，删除调用前的 `if (!worker.Busy())` 门控；默认 `detect_interval_locked=0` 时每个新帧都可覆盖 pending slot。
- 新提交覆盖尚未开始处理的旧 pending frame，并增加 overwritten 计数。
- worker 完成当前推理后立即取最新 pending frame，不等待下一相机帧。
- 使用条件变量替代 1 ms 轮询。
- 停止时唤醒 worker 并安全 join。
- 不建立多帧 FIFO，避免陈旧帧积累。

此调度只提高实际完成吞吐并降低空闲间隙，不改变单次 13.6 ms 延迟。

## 8. VisualTracker

### 8.1 Kalman 状态

首版使用六维状态：

```text
x = [cx, cy, vx, vy, log_width, log_height]
z = [cx, cy, log_width, log_height]
```

中心使用常速度模型，框尺寸在短时预测中保持不变并由测量校正。使用对数尺寸保证宽高始终为正。

每次预测根据实际 `dt` 更新状态转移矩阵：

```text
cx = cx + vx * dt
cy = cy + vy * dt
```

使用 OpenCV `cv::KalmanFilter`，主构建和测试构建增加 OpenCV `video` 组件，不手写矩阵求逆。

### 8.2 单调时间规则

- tracker 内部状态时间只允许向前推进。
- 当前帧 ROI 测量按当前 `capture_done_ts` 校正。
- 重复时间戳不重复预测。
- 倒退时间戳记录错误并拒绝进入普通校正路径。
- RKNN 延迟结果不直接回写已经前进的 Kalman 状态。

### 8.3 RKNN 延迟结果处理

RKNN 结果携带源帧时间。处理规则：

1. SEARCHING 时，只有相对当前帧年龄不超过 50 ms 的检测框可以在其源时间初始化，并预测到当前帧；更老的结果直接丢弃并继续请求最新全图检测。
2. 已 TRACKING 时，用现有速度将检测框平移到当前帧时间，并按帧龄和速度扩大 ROI。
3. 优先在当前相机帧执行 ROI 精修，以当前时间获得新测量。
4. ROI 失败时，只有年龄不超过 50 ms 的 RKNN 框可以作为降权测量；先投影到当前时间，再使用较大的测量协方差。
5. 更老的结果仅用于触发全图重捕获，不校正当前状态。

这样避免实现复杂的 out-of-sequence Kalman 状态回放，同时消除把旧框当作当前框的主要错误。

### 8.4 测量来源与门控

- 高质量 ROI：较小测量噪声。
- 降权 RKNN：较大测量噪声。
- 预测帧：无测量，只执行 predict。
- ROI 分数、四边形几何或创新残差不合格时拒绝校正。
- 使用四维测量的 Mahalanobis 门控，初始阈值采用 99% 卡方门限 13.28，并允许通过配置调整。
- 速度、框尺寸和单帧尺寸比例变化设置物理上限，异常值触发降权或重捕获。

### 8.5 预测四角

Kalman 只估计中心和尺寸，不直接独立预测四个角，避免四角彼此变形。

- 有当前 ROI 四角时，使用当前测量四角。
- 短时预测时，以中心位移平移最后可靠四角，并按预测宽高比例绕中心缩放。
- 预测四角只在最后可靠四角年龄不超过 50 ms 时有效。
- 超过 50 ms 后仍可输出预测 box，但 `has_valid_homography=false`。

## 9. 状态机和失效规则

```text
SEARCHING -> ACQUIRING -> TRACKING -> PREDICT_ONLY -> LOST
     ^            |           |             |          |
     +------------+-----------+-------------+----------+
```

- SEARCHING：只接受全图 RKNN anchor，不发布有效靶面坐标。
- ACQUIRING：获得合格 anchor 后，在 100 ms 内要求至少两次几何一致的当前帧 ROI 测量。该窗口兼容当前约 30 FPS 回归，同时在 120 FPS 下通常会在 20 ms 内完成。
- TRACKING：持续获得 ROI 或合格 anchor 校正。
- PREDICT_ONLY：当前帧无合格测量，但距离最后合格测量不超过 100 ms。
- LOST：最后合格测量年龄超过 100 ms，或连续几何异常，停止发布有效坐标并回到全图搜索。

置信度按真实时间指数衰减，不再每调用一次 `Predict` 就乘固定比例。重捕获时重置异常速度和旧四角，避免轨迹跳跃。

## 10. TargetPlane

### 10.1 四角规范化

- 统一输出左上、右上、右下、左下顺序。
- 靶面三维坐标系原点位于靶心；面对感光纸正面观察时，+X 指向物理右侧，+Y 指向物理上方，+Z 从靶面指向相机，构成右手系。图像坐标仍为 x 向右、y 向下。
- 靶面四角坐标固定为左上 `(-W/2,+H/2)`、右上 `(+W/2,+H/2)`、右下 `(+W/2,-H/2)`、左下 `(-W/2,-H/2)`。圆相位 `phi=0` 位于 +X，正方向为面对靶面时逆时针。
- 验证四边形凸性、面积、边长、对角线、角度和宽高比。
- 靶框物理尺寸作为配置，默认 A4 为 21.0 cm x 29.7 cm，实际比赛靶纸应测量胶带参考边并覆盖默认值。
- 自动判断横放或竖放，但一旦 TRACKING 后不得在两个方向间抖动切换。二维云台不含 roll 轴，设计假定相机 roll 已标定且绝对值小于 90 度；重捕获时结合图像上下方向和上一次有效朝向，禁止四角发生 180 度翻转。若安装无法满足该假定，必须在靶纸增加非对称视觉标记后才能保证圆相位方向。

### 10.2 单应矩阵

使用可靠四角和 `cv::getPerspectiveTransform` 建立：

```text
image pixel <-> target plane centimeter
```

输出前验证：

- 变换矩阵元素有限。
- 四角回投误差满足阈值。
- 目标中心位于四边形内部。
- 相邻帧变换变化不超过运动允许范围。

无有效单应矩阵时，不得输出伪造的厘米坐标。

### 10.3 靶心和半径 6 cm 圆

视觉模块提供纯几何接口：

```cpp
std::optional<cv::Point2f> ProjectTargetPointCm(
    const cv::Point2f& target_point_cm) const;
```

靶心为 `(0, 0)`。圆周点由上层提供相位：

```text
target_point_cm = [6*cos(phi), 6*sin(phi)]
```

视觉模块只负责将物理点映射到当前图像，不负责根据车辆里程生成 `phi`。

## 11. 可选增强

### 11.1 LK 光流

只有以下任一条件成立时进入实现：

- 120 FPS 回放中 ROI 合格率低于 95%。
- ROI 精修 p99 超过 5 ms，导致主循环无法稳定保持 120 Hz。
- 90 度转弯录制中连续两帧以上无法获得四角。

光流使用靶框边缘或内部可靠特征，必须包含前后向误差、RANSAC 单应性和最小有效点数门限。若触发原因是 ROI 精修超出时间预算，则降低完整精修频率，由更轻的光流承担中间帧测量，而不是在每帧精修之外叠加光流。每次可靠 ROI 或 RKNN 重捕获后重置特征，禁止长期无锚累计。

### 11.2 激光光点检测

先录制 405 nm 激光在不同曝光、距离和背景下的视频。只有相机能稳定区分实时光点与感光纸残留痕迹时实现。

若实现，输出实际光点的靶面厘米坐标和置信度，但不在本阶段闭合云台控制环。

## 12. 性能与观测

新增滑动窗口统计：

- 实际 camera FPS。
- camera seq 跳号和主循环丢帧。
- detector submitted、overwritten、started、completed 数量。
- 实际 completed detection Hz。
- preprocess、set、rknn、get、postprocess 时延。
- ROI refine、Kalman、TargetPlane 和主循环总时延。
- source frame age 和 estimate publish age。
- ROI、RKNN、predict-only 各来源帧比例。
- 状态机各状态停留时长和重捕获次数。

至少输出 p50、p95、p99，而不是只输出最后一帧耗时。

## 13. 测试设计

### 13.1 单元测试

- 可变 dt 匀速轨迹的中心和速度误差。
- 30、60、73、120 Hz 输入下时间尺度一致性。
- 随机抖动、重复时间戳和倒退时间戳。
- 连续丢 1、2、6、12 帧时的预测和 LOST 时刻。
- 静止目标噪声下的输出 RMS。
- 急停、反向和 90 度方向变化时的最大过冲。
- Mahalanobis 门控拒绝远离轨迹的错误框。
- 预测 box、宽高和四角始终有限且在边界策略内。

### 13.2 几何测试

- 四角乱序后能够稳定规范化。
- 横放、竖放和透视变形 A4 四边形。
- 使用不参与单应矩阵求解的内部控制点，验证图像到厘米和厘米到图像的独立投影误差；不得只用求解所用四角做自洽往返测试。
- 靶心映射。
- 半径 6 cm 圆在多种单应矩阵下的回投误差。
- 无效、近退化和自交四边形被拒绝。

### 13.3 调度测试

- worker 忙时 pending slot 被最新帧覆盖。
- worker 完成后立即处理最新 pending frame。
- Stop 能唤醒等待线程且不死锁。
- 结果保留正确的源帧 seq 和时间戳。
- 不出现深度大于 1 的图像积压。

### 13.4 录制回放

建立 `vision_replay` 工具，输入视频和可选 CSV 标注，复用与板端相同的 refiner、tracker 和 TargetPlane。

回放集至少包含：

- 静止靶面。
- 直线慢速和快速运动。
- 90 度转弯和运动模糊。
- 急停、反向和短时遮挡。
- 靶面靠近画面边缘。
- 错误矩形干扰。
- 模拟 120 FPS，并按 60 至 73 Hz 注入 RKNN anchor。

### 13.5 板端验证

1. 先在当前 30 FPS 相机上验证功能和无回归。
2. 使用录制的 120 FPS 数据验证主循环预算。
3. 更换 120 FPS 相机后回读实际协商格式和帧率。
4. performance governor 下持续运行 10 分钟。
5. 记录温度、CPU/NPU/DDR 频率和所有 p99 指标。

## 14. 验收标准

功能：

- 所有有效估计都能追溯到源帧 seq 和时间戳。
- 不使用推理结束时间代替测量时间。
- 无测量短帧能够连续预测，超过 100 ms 自动失效。
- 无有效单应矩阵时不输出厘米坐标。

性能：

- 120 FPS 相机实际采集率不低于 118 FPS。
- VisualEstimate 发布率不低于 115 Hz。
- 从主线程被新帧唤醒到发布 VisualEstimate 的完整同步路径，包括帧交接、RKNN 结果消费、ROI、Kalman、TargetPlane、统计和发布，p99 小于 8.33 ms；其中视觉计算目标 p99 小于 7.0 ms，为调度抖动保留余量。
- latest-frame mailbox 下实际 RKNN completed rate 接近模型计算容量，目标不低于 70 Hz。
- 连续运行 10 分钟无队列增长、死锁和热降频导致的持续退化。

精度：

- 靶面中心视觉误差相对独立标注的红色靶心或标定板真值，p95 小于 0.8 cm，最大值小于 1.2 cm；不得用单应矩阵自身定义的原点作为真值。
- 半径 6 cm 圆的纯视觉映射径向误差相对独立标注的圆周控制点，p95 小于 0.5 cm。
- 相对不参与单应求解的独立标注控制点，投影误差 p95 小于 1.5 px。
- 短时预测相对预测帧独立标注中心真值的误差 p95 小于 3 px；下一次有噪声的 ROI 或 RKNN 测量不能作为预测真值。
- 120 FPS 输入下重捕获后 50 ms 内恢复 TRACKING；当前约 30 FPS 回归环境下 100 ms 内恢复 TRACKING，且两种模式均无异常速度尖峰。

## 15. 实施顺序

1. 时间戳、FramePacket、DetectionResult 和统计字段。
2. CameraSource 等待接口和 DetectorWorker latest-frame mailbox。
3. VisualTracker 与主循环接入，保留旧 tracker 作为短期对照。
4. 状态机、时间失效规则和异常门控。
5. TargetPlane 四角规范化、单应矩阵和厘米坐标。
6. 单元测试、调度测试和几何测试。
7. vision_replay 与 120 FPS 数据回放。
8. 当前板端 30 FPS 回归。
9. 120 FPS 相机实测和参数整定。
10. 根据指标决定是否实现 LK 光流和激光光点检测。

每一步都必须保持可构建、可测试，并独立记录性能变化。不得在没有基线数据的情况下同时引入光流、常加速度模型和串口改造。
