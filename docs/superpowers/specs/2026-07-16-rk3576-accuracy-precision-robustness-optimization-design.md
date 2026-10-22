# RK3576 二维云台准度、精确性与鲁棒性优化设计报告

- 日期：2026-07-16
- 状态：设计完成，实施前需建立板端与回放基线
- 审查基线：Git `HEAD=73c7bf9` 加当前未提交工作区
- 审查范围：`cpp_fast_targeting`、运行/诊断脚本、测试、现有设计与实测报告
- 目标平台：RK3576、HF868-2/OV2710、640x480 MJPG、目标 120 FPS、FP16 RKNN

## 1. 结论摘要

当前项目已经形成合理的多速率视觉骨架：相机线程只保留最新帧，RKNN worker 使用深度为 1 的 mailbox，当前帧 ROI 精修承担高频测量，可变 `dt` Kalman 负责短时预测，RKNN 负责全图发现和重捕获。这些设计应保留。

但当前系统还没有把“检测到矩形”闭合成“物理靶心被准确、稳定、可靠地发送给云台”。最高影响的三个问题是：

1. `TargetPlane` 已经算出投影靶心，但它只进入调试显示；串口仍发送轴对齐检测框中心。透视视角下，轴对齐框中心不等于靶面中心，这是系统性准度误差。
2. 新相机为 135 度广角，当前没有镜头畸变、相机内参、相机到云台/激光外参和零位偏置标定；串口只是把 640x480 像素分别线性缩放到 800x480。该映射不能保证等角度，更不能补偿镜头畸变和视差。
3. 现有验证主要证明 FP16 模型能检出训练图，以及 legacy/zero-copy 在当前样本上结果一致；它没有给出独立真值下的靶心误差、静态抖动、运动滞后、误锁率和重捕获时间，因此无法量化“优化后提升了多少”。

推荐采用“标定几何 + 质量感知时序融合 + 带时间语义的控制协议”路线。先修复当前链路的确定性正确性问题并建立回放基线，再升级亚像素四边拟合与投影靶心，随后处理传感器时间戳、角度域跟踪、串口协议和故障恢复。只有独立测试集证明 RKNN 重捕获仍是瓶颈时，才训练四角关键点模型。

本报告不承诺未经实测的百分比收益。每项设计都绑定验收指标，只有 A/B 回放和板端测试通过后才算提升。

## 2. 分析方法与限制

### 2.1 已使用的上下文

- 完整阅读当前 C++ 主链路及头文件：采集、RKNN/RGA、ROI 精修、Kalman、靶面单应性、串口、MJPEG、配置和测试。
- 对照最近设计、教学文档、板端性能报告和启动脚本，核对“设计目标、当前代码、已测数据”三者是否一致。
- 通过 brainstorming skill 对软件优化路线做了方案比较和范围收敛。
- 枚举 MCP resources 与 resource templates；当前会话两者均为空，没有可读取的板端遥测、相机标定、数据集标注或模型元数据。

### 2.2 限制

- 本机不是 RK3576 板端，不能运行 `librknnrt`、RGA、V4L2 相机和实际 STM32 串口闭环。
- 仓库没有独立标注的运动视频、相机内参、曝光参数、云台编码器日志和激光落点真值。
- 当前 README 仍记录旧相机约 27.5 FPS，而配置与启动脚本已经切换到新相机 640x480@120 FPS；脚本也明确要求先验证实际 FPS。因此 120 FPS 是当前请求值，不是本报告认定的实测基线。
- 工作区有大量未提交改动；本报告以当前文件内容为准，不把 `HEAD` 当作实际运行版本。

## 3. “准度、精确性、鲁棒性”的工程定义

仅看模型 `conf`、bbox IoU 或画面是否有框，不足以评价二维云台。

| 维度 | 本项目定义 | 首要指标 |
|---|---|---|
| 准度 Accuracy | 输出靶心/激光落点相对物理真值的系统偏差 | 靶面厘米误差、全视场分区偏差、运动相位滞后 |
| 精确性 Precision | 相同真值下输出的重复性和短时抖动 | 静态二维 RMS、p95 抖动半径、角点/中心帧间跳变 |
| 鲁棒性 Robustness | 光照、运动、遮挡、干扰、设备异常下保持正确输出并安全失效的能力 | 跟踪可用率、误锁率、重捕获时间、陈旧输出时间、恢复时间 |

建议将最终比赛指标分解为以下验收门槛：

1. 视觉几何：相对独立靶心真值，中心误差 p95 小于 0.8 cm、最大值小于 1.2 cm。
2. 静态精确性：固定目标连续 10 秒，靶面中心二维 RMS 小于 0.15 cm，p95 抖动半径小于 0.30 cm。
3. 运动预测：相对同一时刻独立标注，中心预测误差 p95 小于 3 px；不能使用下一帧视觉测量冒充预测真值。
4. 鲁棒性：定义测试集上正确跟踪可用率不低于 99%，30 分钟纯干扰场景误锁为 0，短遮挡后的重捕获 p95 小于 100 ms。
5. 安全失效：最后可信测量超过 100 ms，或预测协方差超过 0.8 cm 等效误差时，不得继续发送有效瞄准命令。
6. 实时性：120 FPS 模式下，当前帧同步视觉路径 p99 小于 8.33 ms；所有输出必须携带可解释的测量年龄。
7. 系统闭环：激光落点相对目标的最终误差满足题目 `D1 <= 2 cm`。视觉指标通过不等于闭环指标自动通过，仍需包含云台和激光的端到端测试。

## 4. 当前数据流与应保留设计

```text
CameraSource latest frame
  FramePacket(frame, capture_done_ts, seq)
        |
        +--> main: tracker.Predict(current frame time)
        |      |
        |      +--> consume latest RKNN result
        |      |      delayed bbox -> velocity projection to current frame
        |      |
        |      +--> current-frame ROI contour refine
        |      |      quad/box/score -> Kalman correction
        |      |
        |      +--> TargetPlane from corners
        |      |      target center currently used only for debug
        |      |
        |      +--> AimWriter(track center/velocity)
        |
        +--> DetectorWorker depth-1 mailbox -> RKNN anchor
```

应保留的部分：

- `FramePacket` 的帧号与单调时间传播，避免把推理结束时刻当测量时刻。
- 相机、主循环和检测线程解耦，只保留最新帧，避免 FIFO 累积陈旧图像。
- RKNN 作为全图 anchor，ROI 精修作为当前帧高频测量，而不是要求 NPU 跑满 120 FPS。
- 可变时间步长 Kalman、来源区分、Mahalanobis 门控、短时置信度衰减。
- FP16 模型作为当前默认；已知 INT8 导出在真实图片上分数恒为 0，不应为了帧率直接恢复。
- zero-copy 不可用时回退 legacy，以及调试流无客户端时不编码。

## 5. 源码审查发现

### 5.1 P0：实施任何调参前应先修复

| 问题 | 代码证据 | 影响 |
|---|---|---|
| 投影靶心未进入控制输出 | `src/main.cpp:47-54` 计算 `target_center_px`；`src/main.cpp:295` 仍调用 `writer.Send(track, ...)`；`src/aim_writer.cpp:120-123` 打包 `snapshot.center` | 透视下产生稳定偏差，准度上限被错误中心定义锁死 |
| 广角像素直接线性缩放 | `include/rkfast/config.hpp:13-21` 声明 135 度镜头；`src/camera_source.cpp:63-76` 只有模式设置；`src/aim_writer.cpp:107-123` 分别按 `sx/sy` 缩放 | 未补偿畸变、视场角、安装偏置与视差，画面不同位置误差不同 |
| 亚像素角点没有用于中心/box | `src/rect_refiner.cpp:117-143` 先用整数 `boundingRect` 生成中心；`src/rect_refiner.cpp:130-131` 才做 `cornerSubPix`；`src/rect_refiner.cpp:178-185` 仍保存旧中心和旧 box | 角点精修收益没有传递到跟踪中心；旋转/透视时 AABB 中心有偏 |
| 图像直角约束与透视模型冲突 | `src/rect_refiner.cpp:133-137` 要求图像角接近 90 度 | 合法的斜视矩形会被拒绝，视角越大越不稳 |
| 45 度附近可能出现重复角点 | `src/rect_refiner.cpp:232-252` 用 sum/diff 极值排序 | 菱形及对称情况可能重复选择同一点，随后几何退化或丢检 |
| prior 选框默认失效 | 默认及启动脚本为 `max_det=1`；`src/rknn_detector.cpp:819-838` 先只留一个候选；`src/rknn_detector.cpp:842-868` 之后才按 prior 打分 | 多目标/干扰时仍只选最高置信度框，易跳目标 |
| 多输出 RKNN 请求索引未赋值 | `src/rknn_detector.cpp:338-345` 未设置 `outputs[i].index=i`；API 在 `third_party/rknn/include/rknn_api.h:433-442` 明确要求 index | 当前单输出模型可能不触发；一旦换多 head/关键点模型会重复请求 output 0 并错配属性 |
| RGA 的真实行距没有传入 | `src/rga_preprocess.cpp:82-88` 把源 stride 写为 `frame_w`、目标 ROI stride 写为 `new_w` | 非连续 Mat 或左右 letterbox 时可能错行、坐标错误或越界 |
| zero-copy 忽略 native stride | `src/rknn_detector.cpp:294-306` 紧密写 FP16；`src/rknn_detector.cpp:508-540` 虽查询并打印 `w_stride/size_with_stride`，却未按 stride 填充 | native tensor 有对齐时输入布局错误，padding 还可能保留旧数据 |
| 数值防护与 `-ffast-math` 冲突 | `CMakeLists.txt:155-166` Release 开启 `-ffast-math`；解码、跟踪、几何大量依赖 `std::isfinite` | 编译器可假定 NaN/Inf 不存在，优化掉关键防护，异常输入可能污染状态 |

### 5.2 P1：直接影响精度或运动鲁棒性

| 问题 | 代码证据 | 影响 |
|---|---|---|
| 单应性“回投验证”不是独立验证 | `src/target_plane.cpp:201-223` 用同一四点精确求解并回投同一四点 | 四点变换理论上本就穿过四个输入点，无法识别边缘拟合偏差、角点噪声和时序跳变 |
| 方向首次猜测后永久锁定 | `src/target_plane.cpp:160-180` 由像面长宽比猜方向；`src/target_plane.cpp:227-230` 锁定；`src/target_plane.cpp:244-249` 失效时不解锁 | 错锁会交换厘米轴；A4 两轴尺度比约为 1.414，丢失后换朝向也不能恢复 |
| 单帧坏四角会立刻清掉 last-good H | `src/target_plane.cpp:110-114` 在验证前先 `Invalidate()` | 一个孤立异常即可让靶面中心闪断，未利用已有 homography timeout |
| 预处理/输出契约靠启发式猜测 | `src/rknn_detector.cpp:625-695` 以维度和 256 魔数猜转置；`src/rknn_detector.cpp:748-787` 按单框数值猜坐标尺度和输出语义 | 更换导出方式、类别数或关键点模型时可静默错解，不一定启动失败 |
| letterbox 反变换只有一个理论 scale | `include/rkfast/rknn_detector.hpp:45-51` 仅存 `scale`；resize 尺寸在 `src/rknn_detector.cpp:583-613` 被取整 | x/y 实际比例与理论 scale 有亚像素差，原图坐标出现稳定小偏差 |
| 最终分数和宽高缺少完整有限性验证 | `src/rknn_detector.cpp:733-811` 未拒绝非正宽高和 NaN score | 负宽高会被 `ClipBox` 交换成表面合法框，NaN 可绕过阈值并破坏排序 |
| RGA 与 OpenCV 插值口径不同 | `src/rga_preprocess.cpp:91` 固定线性插值；`src/rknn_detector.cpp:597-601` 缩小时使用区域插值 | RGA 偶发回退时同一帧族的置信度/框坐标可能跳变 |
| ROI 只偏向暗目标和固定尺度形态学 | `src/rect_refiner.cpp:61-83` 固定 Otsu 反色、3/5 px 核，且只有完全找不到候选才比较 Canny 路径 | 亮目标、低对比、曝光变化、小目标和运动模糊下成功率不稳定 |
| 量测噪声不看质量和年龄 | `src/tracker.cpp:464-488` 只按 `roi/rknn` 选择固定噪声；`src/main.cpp:224-229` 会把旧 RKNN 框投影到当前帧 | 低分 ROI 和年龄接近 50 ms 的 anchor 可能被过度信任 |
| RKNN-only 会周期性重置速度 | `src/tracker.cpp:306-317` acquisition 超时会重新初始化；`src/tracker.cpp:356-359` 只有 ROI 增加连续量测计数 | ROI 失效但 RKNN 仍命中时，tracker 长期停在 ACQUIRING 并周期性把速度归零 |
| 重捕获不能补偿旧帧位置 | `src/tracker.cpp:88-103` 在旧量测时刻以零速度初始化后预测到当前 | 新轨迹没有速度，20-50 ms 检测延迟会原样变成运动位置滞后 |
| 角点预测只做轴向缩放 | `src/tracker.cpp:526-537` 用 `sx/sy` 变换四角 | 目标旋转、透视变化和相机转弯时，四角姿态不能正确预测 |
| 固定 lead_time 不代表真实链路时延 | `include/rkfast/config.hpp:83-85` 默认 50 ms；`src/tracker.cpp:249-251` 只生成 `predict_center`；二进制协议并未发送该点 | 相机、计算、串口和执行器时延变化时会欠补偿/过补偿；当前二进制行为与配置含义不一致 |
| capture_done 不是曝光时间 | `src/camera_source.cpp:140-154` 在 `cap.read()` 返回后打时间戳 | 速度越高，未知采集/解码延迟带来的相位误差越大 |
| ACQUIRING/PREDICT_ONLY 也会发送有效坐标 | `src/tracker.cpp:230-238` 只排除 SEARCHING/LOST；`src/aim_writer.cpp:106-124` 对所有 active 状态打包 | 尚未几何确认或协方差已变大的结果也可能驱动云台 |
| 非阻塞串口写结果被忽略 | `src/aim_writer.cpp:50` 以 `O_NONBLOCK` 打开；`src/aim_writer.cpp:79-87` 忽略短写、EAGAIN 和设备断开 | 可能出现半包、静默丢包，控制端得到间歇性错误数据 |
| 相机/推理无自动恢复状态机 | `src/camera_source.cpp:140-157` 失败后无限 2 ms 重试；`src/rknn_detector.cpp:938-1004` 仅打印推理错误 | USB 断开、驱动卡死、连续 NPU 错误时进程可永久无有效输出但不恢复 |

### 5.3 P2：工程一致性与验证缺口

- `HalfToFloat` 在 `src/rknn_detector.cpp:110-138` 用无符号 exponent 处理 FP16 subnormal，`--exponent` 会下溢；应以参考实现覆盖全部 65536 个半精度 bit pattern。
- `max_predict_misses`、`track_alpha`、`track_beta`、`track_box_alpha`、`predict_decay` 可解析但没有进入当前 Kalman 行为；测试 `tests/rkfast_tests.cpp:83-89` 只检查速度“不增长”，并未验证注释所说的 decay。
- `input_mode_compare` 与 `image_probe` 在 `CMakeLists.txt:169-231` 没有链接/定义 RGA，因此所谓生产路径诊断没有覆盖当前 RGA 预处理。
- `tools/test_rga.sh` 只验证设备、库和初始化，不比较 RGA 与 OpenCV 的像素/检测输出。
- 当前单元测试仅覆盖静态矩形、简单匀速和轴对齐单应性 happy path；没有旋转、透视、畸变、弱光、干扰、丢帧、乱序时间、NaN、串口、相机恢复和状态机边界。
- `benchmark.sh` 仍解析旧的 `infer_fps/pre/post/select` 字段，而主程序输出已经使用 `infer_capacity/detector_hz` 等新字段；它也只评性能，不评准确率。
- README、教学文档、当前配置和启动脚本存在相机分辨率/FPS/默认参数漂移，无法单靠文档确认实际运行 profile。

## 6. 方案比较

### 方案 A：只修现有 bbox + Kalman

内容：修 RGA/RKNN 正确性、将 `max_det` 与候选池解耦、中心改为四角对角线交点、动态测量噪声、串口可靠写、补测试。

优点：改动最小、最快获得确定性收益、无需重训模型。缺点：没有镜头/云台标定时，视场相关系统偏差仍存在；轮廓法在复杂光照和运动模糊下有上限。

结论：作为第一阶段必须做，但不能作为最终方案。

### 方案 B：标定几何 + 四边亚像素精修 + 质量感知时序融合

内容：保留 RKNN bbox 作为 anchor，在标定后的 ROI 中拟合四条边，以多点残差求四角/单应性/协方差；跟踪投影靶心或角度而非 AABB 中心；用真实时间语义和版本化协议连接控制器。

优点：最贴合“已知矩形靶面 + 广角相机 + 高频 ROI + 低频 NPU”的现状；准度、精确性和鲁棒性都有直接机制；模型仍可保持当前 FP16。缺点：需要标定工具、回放数据和 STM32 协议配合，实施量中等。

结论：推荐方案。

### 方案 C：直接训练四角关键点/分割模型

内容：模型直接输出四角或靶面 mask，再做几何和时序融合。

优点：复杂背景、破损边框、部分遮挡下上限更高。缺点：需要高质量四角标注、按视频序列隔离的数据集、RKNN 导出适配和端侧重新验收；模型本身也不能替代镜头、时延和云台标定。

结论：作为方案 B 的可插拔测量源。只有独立测试表明经典四边精修的合格率低于 95%，或重捕获指标不达标时启动。

## 7. 推荐总体架构

```text
V4L2 Capture + Camera Profile
  sensor/dequeue timestamp, exposure, gain, frame seq
       |
       +--------------------> RKNN latest-frame mailbox
       |                         ModelContract -> Top-K anchors
       |
       +--> calibrated ROI / edge bands
              robust four-line fit + subpixel edge localization
              -> QuadMeasurement(corners, projective center, covariance, quality)
                        |
                        +--> association + fixed-lag/adaptive CV tracker
                               TargetState(angle, angular rate, quad, H, covariance)
                                      |
                                      +--> latency alignment + boresight correction
                                             AimCommand(valid, age, angle, rate, quality)
                                                    |
                                                    +--> protocol v2 / v1 compatibility

Every stage -> structured telemetry -> event ring buffer -> vision_replay
```

核心原则：

1. bbox 只负责发现和 ROI，不再定义最终靶心。
2. 靶心由经畸变校正的四边/四角几何定义，跟踪和控制使用同一个中心口径。
3. 每个量测携带时间、质量和协方差；“有值”不等于“可用于驱动”。
4. 最终控制在角度域或标定后的虚拟相机域工作，不直接依赖原始像素缩放。
5. 所有优化先在确定性 replay 上 A/B，再上板端闭环；一次只改变一个主变量。

## 8. 详细设计

### 8.1 相机标定与坐标体系

新增版本化 `CameraProfile`：

```text
camera identity + resolution + fourcc + focus/lens profile
K, distortion model, distortion coefficients
camera-to-gimbal rotation/translation
laser/gimbal zero offset or boresight residual map
calibration date, holdout reprojection error, file checksum
```

实施规则：

- 使用 ChArUco/高质量标定板覆盖画面中心、四边和四角，并覆盖实际工作距离；按独立 holdout 图比较 OpenCV rational pinhole 与 fisheye 模型，选择外推误差更小者，不能仅看训练重投影误差。
- RKNN 保持原始图输入，避免改变模型训练域。ROI 精修对预测区域生成局部校正图，或将边缘点用 `undistortPoints` 转到归一化相机平面后拟合。
- 从归一化射线计算 yaw/pitch，再应用相机到云台外参和激光零位补偿。135 度视场下，不再假设像素偏移与角度成线性关系。
- 标定 profile 与分辨率、镜头、焦距设置严格绑定。profile 缺失、校验失败或相机实际模式不匹配时进入 `UNCALIBRATED`，允许调试显示但默认不输出比赛级有效控制命令。
- 将“靶面中心”和“激光应指向的像素”分开。相机与激光不共轴时，需要通过多距离标定或靶面距离估计补偿视差。

### 8.2 RKNN 与预处理契约

为模型增加显式 `ModelContract`，启动时校验：

- 模型 SHA-256、输入 shape/layout/type、RGB/BGR、数值范围、letterbox 颜色和插值方式。
- 输出数量、每个 output 的 index/shape/type/layout、坐标格式、是否归一化、是否有 objectness、类别数和附加字段。
- 当前模型不满足契约时直接启动失败，不再以输出数值或 256 魔数猜语义。

正确性改造：

1. 对每个 `rknn_output` 设置 `index=i`。
2. `PreprocessMeta` 保存 `new_w/new_h/pad_x/pad_y/scale_x/scale_y`，用实际取整后的比例反变换。
3. RGA 使用真实 `cv::Mat.step` 和目标 buffer stride；zero-copy 按 native `w_stride/h_stride` 写入并清零 padding。不满足已支持布局时自动、可观测地回退 legacy。
4. RGA 与 OpenCV 使用同一插值口径。板端对多种横纵比、非连续 Mat、边缘图案做逐像素和最终检测差异测试。
5. 对输入类型、尺寸、连续性和所有输出坐标/分数做有限性与范围检查；异常只产生带原因的失败结果，不允许 OpenCV 异常穿出 worker 线程。

关联改造：

- NMS 后保留 Top-K 候选，例如 K=10；`max_output_targets=1` 只在轨迹关联后生效。
- 关联代价使用归一化创新距离、IoU、中心距离、检测分数和后续几何分数，而不是只用最高 conf。
- 已锁定时，候选必须通过 Kalman gate 和几何验证；切换目标要求连续多帧优势，避免单帧干扰导致跳锁。

### 8.3 四边亚像素精修与投影靶心

推荐替换“整 ROI 二值化后找任意四边形”为“prior 引导的四条边测量”：

```text
predicted quad/box
 -> distortion-aware ROI
 -> four edge bands
 -> gradient samples along edge normals
 -> robust line fit (RANSAC initialization + Huber/Tukey refinement)
 -> adjacent line intersections
 -> multi-point homography refinement
 -> projective target center + covariance + quality
```

具体要求：

- 同时支持亮边/暗边，以梯度强度和方向为主要证据，不固定 `THRESH_BINARY_INV`。
- 核尺寸、搜索带宽和最小边缘支持按目标尺度变化，不使用固定 3/5 px 覆盖所有距离。
- 图像透视下不要求四个角是 90 度；在去畸变/靶面模型中检查物理宽高比、凸性、边缘顺序、对边一致性、面积和条件数。
- 最终中心使用靶面 `(0,0)` 的投影，即四边形两条对角线交点/优化后单应性的目标原点，不使用 `boundingRect` 中心或四角简单平均。
- 单应性用四条边上的多点联合估计；验证残差必须来自未直接决定四角的边缘样本或独立控制点。增加 condition number、中心是否在多边形内、相邻帧几何变化和 orientation continuity 检查。
- 输出质量至少包含：四边支持率、边缘对比度、线拟合 RMS、最小交角、物理宽高比误差、独立重投影残差和 ROI 边界截断比例。
- 根据线拟合残差和交点几何传播得到中心协方差；该协方差进入 tracker，而不是只把启发式 score 当概率。
- 单帧新 homography 验证失败时保留未超时的 last-good 结果并增加年龄/协方差；真正重捕获或目标方向变化时显式重置 orientation 锁。

保留当前 contour/Otsu/Canny 作为 fallback 候选，但二值与边缘路径应统一评分后择优，不能在二值路径找到一个勉强候选后就停止比较。

### 8.4 时间同步、跟踪与状态机

短期仍可使用自适应常速度模型，不建议立即切常加速度。改造重点是坐标、时间和协方差：

- 跟踪中心从原始 AABB 像素改为去畸变归一化坐标或 yaw/pitch，状态为 `[angle_x, angle_y, rate_x, rate_y, log_scale_w, log_scale_h]`。pixel quad 作为 ROI 状态单独维护。
- 过程噪声根据 NIS、运动速度、拐弯/急停和可选云台编码器变化自适应；创新连续偏大时进入 maneuver 模式，提高过程噪声，而不是沿旧速度长期外推。
- 测量噪声由来源、中心协方差、ROI 质量、RKNN conf、量测年龄和是否触边共同决定。陈旧 RKNN anchor 的噪声随年龄和速度增大。
- RKNN-only 合法量测也必须推进 acquisition 计数，不能因 ROI 暂时失效周期性重置滤波器。
- 重捕获先在当前帧 ROI 做几何确认；只能使用旧 RKNN 框时，保持“位置未知速度”的高协方差状态，而不是假装旧帧位置就是当前位置。
- 使用 100 ms 左右状态/量测 ring buffer。若回放数据证明简单投影仍有明显偏差，再实现固定延迟 out-of-sequence update：在源时间校正后按已保存的 ROI 测量重放到当前时刻。
- 从原生 V4L2 buffer 获取单调 sensor/dequeue timestamp；拿不到曝光中点时，至少标定 `capture_done_ts` 到曝光中点的固定偏置和抖动，并写入 telemetry。

状态机改为：

```text
SEARCHING -> VERIFYING -> TRACKING -> COASTING -> LOST
                 |            |          |
                 +------------+----------+
```

- `VERIFYING`：RKNN anchor 后要求至少两次连续几何测量、正确候选关联和协方差达标；此时默认不驱动激光。
- `TRACKING`：当前或短龄测量有效，协方差在门限内。
- `COASTING`：无新测量但预测仍在误差预算内；是否有效由“预测协方差对应厘米误差”决定，不按固定 miss 数决定。
- `LOST`：年龄或协方差超限立即发 invalid/stop；重捕获后清除旧速度、旧四角和旧 orientation。

`max_predict_misses`、旧 alpha-beta 参数和 `predict_decay` 要么真正接入明确定义的策略，要么删除，避免配置看似生效但实际无效。

### 8.5 控制输出与串口协议

当前 v1 协议可保留兼容模式，但最终推荐 v2 固定 24 字节：

| 字节 | 字段 | 说明 |
|---|---|---|
| 0-1 | SOF `AA 55` | 双字节帧头 |
| 2 | version=2 | 协议版本 |
| 3 | length=24 | 固定长度 |
| 4 | flags | valid、predicted、calibrated、fault |
| 5 | state | VERIFYING/TRACKING/COASTING/LOST |
| 6-7 | seq | 帧序号 |
| 8-9 | age_ms | 当前时刻到测量时刻 |
| 10-11 | horizon_ms | 控制器预测使用的执行时域 |
| 12-13 | yaw_error_mrad | 标定后的水平角误差 |
| 14-15 | pitch_error_mrad | 标定后的垂直角误差 |
| 16-17 | yaw_rate_mrad_s | 水平角速度 |
| 18-19 | pitch_rate_mrad_s | 垂直角速度 |
| 20 | confidence | 0-255，来自协方差/质量映射 |
| 21 | reserved | 固定 0 |
| 22-23 | CRC16-CCITT | 覆盖 2-21 |

时间补偿只能有一个最终责任方。推荐视觉端发送带 age 的角度状态，STM32 在控制 tick 根据 `age + servo_delay` 做最后一次预测；视觉端不再同时发送已 lead 的位置让 MCU 再用速度二次补偿。

若短期不能改 STM32：

- v1 兼容模式应明确配置 `prediction_owner=vision|mcu`。
- `vision` 模式发送标定后的预测位置并将速度置零；`mcu` 模式发送当前状态和速度，视觉不预加 lead。
- 800x480 必须定义成有内参的“虚拟相机”，从角度投影到虚拟像素；不能继续简单使用独立 `sx/sy`。

串口实现使用有界 `WriteAll`：处理 EINTR、短写、EAGAIN 和断开，超时后丢弃整帧并计数，不阻塞主视觉循环。增加序号跳变、CRC 错误、发送失败和重连计数。invalid/stop 帧也必须周期发送，形成控制器 watchdog。

### 8.6 相机成像稳定性

软件应通过原生 V4L2 control 显式设置并回读：

- 自动曝光关闭、固定曝光时间、固定增益、固定白平衡和固定焦距状态。
- 目标是曝光时间满足运动模糊预算，而不是只追求画面亮；用补光换短曝光。
- 每帧或低频记录亮度分位数、饱和比例、边缘梯度和模糊指标，区分“算法丢失”和“图像已无可用边缘”。
- 启动时验证实际宽高/FPS/FOURCC 和关键 control。与 profile 不一致时明确降级或启动失败，不允许静默运行在 30 FPS/自动曝光。

如果 HF868-2/OV2710 为滚动快门路径，应先通过高速运动标定行时序误差。短曝光仍有明显梯形/弯曲时，再结合曝光行时序或 IMU/云台角速度做补偿；在证据出现前不引入复杂模型。

### 8.7 故障恢复与可观测性

增加三个 watchdog：

1. Camera watchdog：超过可配置时限无新帧，关闭并重开设备，指数退避；恢复后重新验证模式和 controls，并重置 tracker。
2. Detector watchdog：连续推理/RGA 错误达到门限，先锁存 OpenCV/legacy fallback；仍失败则重建 RKNN context，期间输出 invalid。
3. Output watchdog：串口连续发送失败则关闭重开；控制端超过 100 ms 未收到有效 CRC 帧必须停机或保持安全状态。

每次运行输出一份 machine-readable manifest：Git commit/dirty 标记、模型 hash、配置 hash、标定 hash、RKNN/RGA/内核版本、实际相机参数和 CPU/NPU/DDR governor。

Telemetry 至少包含：

- capture/dequeue/infer/refine/update/publish/send 时间戳与 p50/p95/p99。
- frame seq 跳号、worker overwrite、结果年龄、测量年龄和输出年龄。
- 候选数、选中关联代价、ROI 质量分解、NIS、协方差、拒绝原因。
- 状态停留时长、误锁/切换、重捕获、camera/NPU/RGA/serial 错误和恢复次数。
- 事件触发环形录像：LOST、gate reject 连续发生、模式降级、设备重连时保存前后数秒帧与 JSONL。

MJPEG/日志编码不得进入实时关键路径。调试客户端存在时也应限帧，例如 10-15 FPS，并在性能报告中标记 debug 状态。

### 8.8 数据与模型策略

现有“训练图片 20/20 命中”和“207/207 输入路径一致”不能作为泛化准确率。建立按录制序列划分的数据集，禁止相邻视频帧跨 train/val/test：

- 距离、画面九宫格位置、横竖姿态、斜视角、亮暗、曝光不足、MJPG 压缩、运动模糊、目标触边、部分遮挡。
- 相似矩形、纸张、窗框、屏幕、无目标场景等 hard negatives。
- 直线、匀速、急停、反向和方形轨迹 90 度转弯。
- 原始帧标注四角、物理靶心和可见性；模糊到无法确定真值的帧应标为不可观测，而不是强行给精确角点。

只有以下任一条件出现才训练关键点模型：

- 标定与四边精修后，ROI 合格率仍低于 95%。
- 遮挡/复杂背景下重捕获 p95 超过 100 ms。
- hard-negative 误锁无法通过几何验证和关联解决。
- 四边精修 p99 超过 5 ms，且 KLT/局部优化也不能满足 120 Hz。

关键点模型仍需输出质量/可见性，并保留经典边缘 refinement 作亚像素校正与独立一致性检查。

## 9. 文件级改造边界

| 模块 | 主要改造 |
|---|---|
| `include/rkfast/types.hpp` | 明确 `QuadMeasurement`、`TargetState`、`AimCommand`、时间与协方差字段，使用 enum 替代自由字符串状态 |
| `camera_source.*` | 原生 V4L2 时间戳/controls、实际模式硬校验、故障恢复、采集统计 |
| 新增 `camera_profile.*` | 标定加载、hash 校验、点/射线/角度转换、虚拟相机投影 |
| `rga_preprocess.*` | step/stride 正确处理、统一插值、失败锁存与 parity 统计 |
| `rknn_detector.*` | ModelContract、output index、显式 schema、精确 letterbox、Top-K 候选、完整有限性检查 |
| `rect_refiner.*` | distortion-aware edge bands、四边鲁棒拟合、投影中心、质量与协方差 |
| `target_plane.*` | 多边缘点优化、独立残差、条件数、时序连续性、重捕获时 orientation 重置 |
| `tracker.*` | 角度域/归一化域状态、质量感知 R、自适应 Q、协方差有效性、可选固定延迟回放 |
| `aim_writer.*` | AimCommand、v2 CRC16、有界 WriteAll、v1 兼容语义、发送/恢复统计 |
| `main.cpp` | 只编排数据流；projective center 真正进入 tracker/output；watchdog 和 structured stats |
| 新增 `tools/vision_replay.cpp` | 复现相同帧序列/时间戳，输出逐帧指标和 A/B 差异 |
| 新增 `tools/calibrate_camera.cpp` | 生成/验证 CameraProfile 和 holdout 报告 |
| `tests` | 数值、几何、状态机、协议、回放和故障注入测试 |
| `benchmark.sh` | 同步当前字段，增加 p50/p95/p99 与准确率报告，不再只算平均性能 |

为避免 `main.cpp` 继续膨胀，标定、测量、状态估计和输出协议保持独立接口；不做与目标无关的全项目重构。

## 10. 分阶段实施与门禁

### 阶段 0：基线与可复现性

1. 录制静态九宫格、方形运动、遮挡、弱光和 hard-negative 数据，保存相机实际参数。
2. 实现最小 `vision_replay` 和逐帧 JSONL；冻结当前工作区、模型和配置 hash。
3. 用独立真值输出当前 accuracy/precision/robustness 基线。

门禁：同一输入重复运行输出确定性一致；没有基线不得宣称后续优化有效。

### 阶段 1：P0 正确性

1. 修 RKNN output index、显式 schema、finite/正宽高验证、精确 letterbox。
2. 修 RGA 与 zero-copy stride，统一插值，补 RGA/OpenCV parity。
3. 去掉关键数学模块的 `-ffast-math`，修 FP16 subnormal 转换。
4. 解耦 Top-K 候选与最终单目标输出。
5. 用四角投影中心替代 AABB 中心，但先以 feature flag 做 replay 对照。

门禁：旧验证集检测无回归；RGA/legacy/zero-copy 中心差 p95 小于 0.2 px；坏输入全部 fail closed；projective center 在合成透视测试中恢复真中心。

### 阶段 2：几何准度与静态精确性

1. 完成 CameraProfile、全视场标定和虚拟相机。
2. 实现四边亚像素拟合、质量分解、协方差和多点 homography。
3. tracker 与 AimWriter 统一使用投影靶心/角度。
4. 完成激光/云台零位及多距离视差标定。

门禁：独立静态集达到第 3 节视觉几何与抖动门槛；全视场没有明显残差趋势。

### 阶段 3：运动精度与鲁棒性

1. 获取曝光相关时间戳或标定时间偏置；按测量质量/年龄设置 R，自适应 Q。
2. 实现 VERIFYING/COASTING 和协方差驱动的 valid；必要时增加固定延迟更新或 KLT 初值。
3. 上线协议 v2、唯一 prediction owner、相机/NPU/串口 watchdog。
4. 运行 120 FPS、热机、调试流开关、设备重连和串口压力测试。

门禁：运动预测、可用率、误锁、重捕获、安全失效和 p99 实时性全部通过。

### 阶段 4：模型增强

只有阶段 3 的失败归因明确指向 detector/refiner 泛化能力时，才补数据并训练四角关键点模型。保持当前 FP16 bbox 模型作为回滚基线，比较完整系统指标而不是只比较 mAP。

## 11. 验证矩阵

| 场景 | 真值/方法 | 必看指标 |
|---|---|---|
| 静态九宫格 x 多距离 | 临时 ChArUco/独立高精度靶心 | 分区 bias、厘米 p50/p95/max、抖动 RMS |
| 合成透视/畸变四边形 | 已知 homography 与中心 | corner/center error、退化拒绝、顺序稳定 |
| 匀速/加减速/90 度转弯 | 高帧率标注或独立参考相机 | 时间对齐误差、滞后、过冲、NIS |
| 遮挡 1-12 帧 | 带可见性真值的 replay | COASTING 误差、invalid 时刻、重捕获时间 |
| 弱光/过曝/双极性边框 | 固定真值与成像统计 | ROI 合格率、质量校准、误拒绝 |
| hard negatives/多矩形 | 无目标或目标 ID 真值 | false lock、ID switch、Top-K 关联 |
| RGA/OpenCV/legacy/zero-copy | 同帧像素与输出对比 | checksum、最大像素差、中心/置信度差 |
| 异常数值与模型契约 | 单元/属性测试 | fail closed、无 NaN 泄漏、明确错误 |
| 相机拔插/NPU 连续错误 | 板端故障注入 | invalid 延迟、恢复时间、无死锁 |
| 串口短写/丢字节/CRC 错 | PTY/MCU loopback | 半包处理、序号、CRC、watchdog |
| 30 分钟热机 | 板端连续运行 | p99、温度/频率、漂移、错误与恢复计数 |

A/B 规则：使用完全相同的帧、原始时间戳、配置和初始状态；报告每个序列以及汇总分位数，不能只展示最佳片段或平均值。

## 12. 风险、回滚与不采用项

- 标定文件错误会比无标定更危险：profile 必须带设备/分辨率/hash 绑定，错误时 fail closed。
- 四边拟合在边框破损时可能失败：保留 contour 和 RKNN fallback，但 fallback 降低有效等级，不伪装成高精度量测。
- 角度域 tracker 是坐标口径变化：通过 replay 同时运行旧 pixel tracker 和新 tracker，直到全场景门禁通过。
- 协议 v2 需要 STM32 配合：上线期双协议可选，但每次运行只启用一种 prediction owner。
- KLT 和固定延迟回放会增加状态复杂度：只有数据证明简单方案不达标时启用。
- 不采用“继续盲调 conf/Kalman 参数”作为主路线，因为它不能修复中心定义、畸变、时间和协议错误。
- 不采用“先换更大模型”作为主路线，因为当前主要准度瓶颈在几何/标定/控制接口，模型升级无法消除这些系统误差。

## 13. 最终决策

推荐顺序为：

```text
可复现基线
 -> P0 数据契约与数值正确性
 -> 广角/云台/激光标定
 -> 四边亚像素测量与投影靶心
 -> 质量感知时序融合
 -> 带 age/CRC 的控制协议与 watchdog
 -> 仅在指标仍失败时升级关键点模型
```

这条路线直接对应三类目标：标定和投影靶心提高准度，四边亚像素拟合与协方差融合提高精确性，Top-K 关联、状态机、故障恢复和版本化协议提高鲁棒性。最终是否“有提升”只由第 3 节门槛和第 11 节 A/B 数据判定。
