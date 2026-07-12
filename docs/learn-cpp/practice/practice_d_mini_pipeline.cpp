/******************************************************************************
 * 练习 D: 微型视觉管线模拟 (组合 A+B+C 全部知识点)
 *
 * 这个练习对应真实项目的完整架构, 每个类映射到真实的文件:
 *
 *   本练习的类              真实项目文件
 *   ──────────────────────  ───────────────────────────────────
 *   Frame 结构体            types.hpp — 项目里用 cv::Mat, 这里用自定义结构体
 *   BBox 结构体             types.hpp — struct Box + BoxWidth/BoxIou 等
 *   CameraSource 类         camera_source.cpp — 相机采集线程
 *   Detector 类             rknn_detector.hpp — NPU 推理 + DetectorWorker
 *   ROI_Refiner 类          rect_refiner.cpp — ROI 精修算法
 *   Tracker 类              tracker.cpp — α-β 滤波器 + 状态机
 *   OutputWriter 类         aim_writer.cpp — $AIM 格式串口输出
 *   Pipeline 类             main.cpp — 串联所有组件的主循环
 *
 * 数据流向 (对应真实项目的 main.cpp 主循环):
 *
 *   CameraSource ──帧──▶ Detector ──检测框──▶ ROI_Refiner ──精修框──▶ Tracker
 *                                                                       │
 *                                                          TrackSnapshot │
 *                                                                       ▼
 *                                                                 OutputWriter
 *                                                                       │
 *                                                              "$AIM,..." │
 *                                                                       ▼
 *                                                                  终端输出
 *
 * 本练习覆盖的知识点:
 *   - 类组合 (Composition): Pipeline 包含所有组件
 *   - 依赖注入 (Dependency Injection): 通过引用传递共享数据
 *   - 多线程并发: CameraSource 有采集线程
 *   - std::vector 批量操作: 存储多个检测结果
 *   - 时间管理: std::chrono 模拟实时系统
 *   - IOU 计算与目标选择: 练习 A 的知识
 *   - α-β 滤波与状态机: 练习 B 的知识
 *   - 线程安全的 Buffer: 练习 C 的知识
 *   - 格式化输出: std::ostringstream + std::snprintf
 *   - 统计收集: 帧率、延迟、状态计数
 *
 * 编译: g++ -std=c++17 -pthread -o practice_d_mini_pipeline practice_d_mini_pipeline.cpp
 * 运行: ./practice_d_mini_pipeline
 ******************************************************************************/

#include <algorithm>   // std::max, std::min
#include <atomic>      // std::atomic<bool>
#include <chrono>      // std::chrono 时间库
#include <cmath>       // std::abs, std::round
#include <cstdint>     // uint64_t
#include <iomanip>     // std::fixed, std::setprecision
#include <iostream>    // std::cout, std::endl
#include <mutex>       // std::mutex, std::lock_guard
#include <optional>    // std::optional<T>
#include <random>      // std::mt19937, std::uniform_real_distribution
#include <sstream>     // std::ostringstream (字符串流, 用于格式化输出)
#include <string>      // std::string
#include <thread>      // std::thread, std::this_thread::sleep_for
#include <vector>      // std::vector<T>

/******************************************************************************
 * Frame 结构体 —— 模拟图像帧
 *
 * 对应真实项目: cv::Mat (OpenCV 的图像数据结构)
 *
 * 这里不依赖 OpenCV, 用简化的结构体表示一帧图像。
 * 真实项目中 Frame 包含完整的像素数据 (宽度*高度*3 字节的 RGB 数据)。
 ******************************************************************************/
struct Frame {
    int frame_id;       // 帧编号 (每帧唯一, 单调递增)
    double timestamp;   // 采集时间戳 (秒)
    int width;          // 图像宽度 (像素)
    int height;         // 图像高度 (像素)

    // 构造函数
    Frame(int id, double ts, int w, int h)
        : frame_id(id), timestamp(ts), width(w), height(h) {}

    // 打印摘要
    void print() const {
        std::cout << "Frame#" << frame_id
                  << " @t=" << timestamp
                  << " " << width << "x" << height;
    }
};

/******************************************************************************
 * BBox 结构体 —— 边界框 (Bounding Box)
 *
 * 对应真实项目: types.hpp 里的 struct Box
 *
 * 复用练习 A 中的 Box 概念, 增加了:
 *   - 置信度 (confidence) — 检测器对检测结果的信心
 *   - 类别 ID (class_id) — 检测到的目标类别
 *   - IOU 计算 (非成员函数)
 ******************************************************************************/
struct BBox {
    float x1, y1, x2, y2;  // 边界框坐标
    float conf;              // 置信度 (0.0 ~ 1.0)
    int class_id;            // 类别 ID (本项目只检测一个类别, 固定为 0)

    BBox() : x1(0), y1(0), x2(0), y2(0), conf(0), class_id(0) {}

    BBox(float x1_, float y1_, float x2_, float y2_, float conf_)
        : x1(x1_), y1(y1_), x2(x2_), y2(y2_), conf(conf_), class_id(0) {}

    float width()  const { return x2 - x1; }
    float height() const { return y2 - y1; }
    float area()   const { return width() * height(); }
    float center_x() const { return (x1 + x2) * 0.5f; }
    float center_y() const { return (y1 + y2) * 0.5f; }

    void print(const char* label = "") const {
        std::cout << label << " BBox(" << x1 << "," << y1
                  << " " << x2 << "," << y2
                  << ") conf=" << conf
                  << " center=(" << center_x() << "," << center_y() << ")";
    }
};

// IOU 计算 (非成员函数, 操作两个 BBox)
// 和练习 A 的 IOU 完全一样的算法
float calc_iou(const BBox& a, const BBox& b) {
    float ix1 = std::max(a.x1, b.x1);
    float iy1 = std::max(a.y1, b.y1);
    float ix2 = std::min(a.x2, b.x2);
    float iy2 = std::min(a.y2, b.y2);
    float iw  = std::max(0.0f, ix2 - ix1);
    float ih  = std::max(0.0f, iy2 - iy1);
    float inter = iw * ih;
    if (inter <= 0.0f) return 0.0f;
    float uni = a.area() + b.area() - inter;
    return inter / std::max(uni, 1e-6f);
}

/******************************************************************************
 * CameraSource 类 —— 相机帧采集器
 *
 * 对应真实项目: camera_source.cpp
 *
 * 真实项目中 CameraSource:
 *   - 启动一个独立线程 (CaptureLoop) 不断从摄像头读取帧
 *   - 帧存入 mutex 保护的共享缓冲区 (cv::Mat latest_)
 *   - 主线程通过 ReadLatest() 获取最新帧
 *   - 序列号 (seq_) 用于判断是否为新帧
 *
 * 这里简化:
 *   - 用独立的采集线程模拟摄像头
 *   - 每 200ms 产生一帧 (5fps) — 方便观察, 真实项目是 30~120fps
 *   - 目标在画面中 "移动" (模拟匀速运动 + 随机抖动)
 *   - 帧存入线程安全的缓冲区
 ******************************************************************************/
class CameraSource {
public:
    CameraSource(int width, int height)
        : width_(width), height_(height)
        , running_(false), frame_id_(0), seq_(0), has_frame_(false) {}

    ~CameraSource() { stop(); }

    // 启动采集线程
    // 真实项目: CameraSource::Open()
    void start() {
        running_.store(true);
        capture_thread_ = std::thread(&CameraSource::run, this);
        std::cout << "[CameraSource] 采集线程已启动 (模拟 5fps)" << std::endl;
    }

    // 停止采集线程
    // 真实项目: CameraSource::Close()
    void stop() {
        running_.store(false);
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        std::cout << "[CameraSource] 采集线程已停止" << std::endl;
    }

    // 读取最新帧 (非阻塞)
    // 返回 optional<Frame>: 有帧则返回, 无帧返回 nullopt
    // 真实项目: CameraSource::ReadLatest()
    std::optional<Frame> readLatest() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_frame_) return std::nullopt;
        if (last_read_seq_ == seq_) return std::nullopt;  // 不是新帧
        last_read_seq_ = seq_;
        return latest_frame_;
    }

    // 获取当前的 "真实" 目标位置 (仅用于模拟检测器评估)
    // 真实项目当然没有这个, 因为不知道真实位置
    // 这里暴露出来是为了让模拟检测器可以加噪声
    float true_target_x() const { return true_target_x_; }
    float true_target_y() const { return true_target_y_; }

private:
    // 采集线程的主循环
    // 真实项目: CameraSource::CaptureLoop()
    void run() {
        // 模拟目标在画面中移动: 从画面左上角移动到右下角
        // 真实目标运动: 每秒在 x 方向移动 40 像素, y 方向移动 25 像素
        true_target_x_ = width_ * 0.3f;   // 从左侧 30% 处开始
        true_target_y_ = height_ * 0.3f;  // 从上侧 30% 处开始

        // 用 steady_clock 获取启动时间
        auto start_time = std::chrono::steady_clock::now();

        while (running_.load()) {
            // 等待一帧的时间 (5fps → 每帧 200ms)
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            if (!running_.load()) break;  // 双重检查

            // 计算时间戳
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - start_time).count();

            // 移动目标 (匀速 + 到达边界反弹)
            // 真实项目中这个不存在, 这里纯粹是为了让模拟更 "真实"
            float dt = 0.2f;  // 200ms
            true_target_x_ += 40.0f * dt;
            true_target_y_ += 25.0f * dt;

            // 边界反弹 (超出画面就反向)
            if (true_target_x_ > width_ - 50 || true_target_x_ < 50) {
                true_target_x_ = std::max(50.0f, std::min(true_target_x_, (float)width_ - 50));
            }
            if (true_target_y_ > height_ - 50 || true_target_y_ < 50) {
                true_target_y_ = std::max(50.0f, std::min(true_target_y_, (float)height_ - 50));
            }

            // 创建帧
            Frame frame(++frame_id_, elapsed, width_, height_);

            // 写入共享缓冲区 (加锁)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_frame_ = frame;
                ++seq_;
                has_frame_ = true;
            }
        }
    }

    // 配置
    int width_, height_;

    // 线程控制
    std::atomic<bool> running_;
    std::thread capture_thread_;

    // 共享数据 (受 mutex 保护)
    mutable std::mutex mutex_;
    Frame latest_frame_{0, 0.0, 0, 0};  // 默认构造
    uint64_t seq_ = 0;
    uint64_t last_read_seq_ = 0;
    bool has_frame_ = false;
    int frame_id_ = 0;

    // 模拟的真实目标位置 (仅供学习, 真实项目无此字段)
    float true_target_x_ = 0.0f;
    float true_target_y_ = 0.0f;
};

/******************************************************************************
 * Detector 类 —— 模拟 AI 检测器
 *
 * 对应真实项目: rknn_detector.hpp (RknnDetector + DetectorWorker)
 *
 * 真实项目中 RknnDetector:
 *   - 加载 .rknn 模型文件 (RK3576 NPU 专用格式)
 *   - 对输入帧做预处理 (resize, normalize, RGA 硬件加速)
 *   - 调用 rknn_run() 在 NPU 上推理
 *   - 后处理: 解析输出张量, 得到边界框列表
 *   - 选择最优目标 (按面积、置信度、与 PriorBox 的 IOU)
 *
 * 这里用随机数模拟:
 *   - 60% 概率检测到目标 (模拟模型精度)
 *   - 检测位置 = 真实位置 + 随机噪声 (±15 像素)
 *   - 检测框大小 = 真实大小 + 随机抖动
 ******************************************************************************/
class Detector {
public:
    Detector() : rng_(std::random_device{}()) {}

    // 检测一帧
    // 参数: frame — 输入帧 (真实项目是 cv::Mat)
    //       true_x, true_y — 真实目标位置 (只有模拟中才知道)
    // 返回: optional<BBox> — 检测到的边界框, 或 nullopt (没检测到)
    //
    // 对应真实项目: RknnDetector::Infer()
    std::optional<BBox> detect(const Frame& /*frame*/,
                                float true_x, float true_y) {
        // 60% 概率 "检测到" (模拟 YOLO 的 recall)
        std::uniform_real_distribution<float> detect_chance(0.0f, 1.0f);
        if (detect_chance(rng_) > 0.6f) {
            // 40% 概率检测失败 (漏检 — false negative)
            return std::nullopt;
        }

        // "检测到" 了。模拟检测框: 真实位置 + 噪声
        // 噪声范围: ±15 像素 (模拟模型的位置误差)
        std::uniform_real_distribution<float> noise(-15.0f, 15.0f);
        float cx = true_x + noise(rng_);
        float cy = true_y + noise(rng_);

        // 检测框大小: 70~90 像素宽, 70~90 像素高 + 随机抖动
        std::uniform_real_distribution<float> size_noise(-10.0f, 10.0f);
        float half_w = 40.0f + size_noise(rng_);  // 半宽 ≈ 35~45
        float half_h = 40.0f + size_noise(rng_);  // 半高 ≈ 35~45

        // 置信度: 0.6 ~ 1.0 之间随机 (模拟模型输出)
        std::uniform_real_distribution<float> conf_range(0.6f, 1.0f);

        return BBox(
            cx - half_w, cy - half_h,
            cx + half_w, cy + half_h,
            conf_range(rng_)
        );
    }

    // 从多个检测结果中选择最佳目标
    // 对应真实项目: RknnDetector::ChooseDetection()
    // 策略: 选择置信度最高的那个
    std::optional<BBox> select_best(const std::vector<BBox>& candidates) {
        if (candidates.empty()) return std::nullopt;

        const BBox* best = &candidates[0];
        for (const auto& box : candidates) {
            if (box.conf > best->conf) {
                best = &box;
            }
        }
        return *best;
    }

private:
    // 随机数生成器
    // std::mt19937 是梅森旋转算法, 比 C 语言的 rand() 好很多:
    //   1. 周期极长 (2^19937-1), 不会很快重复
    //   2. 分布均匀
    //   3. 线程安全 (每个线程一个实例)
    // std::random_device 提供真正的随机种子 (硬件熵源)
    std::mt19937 rng_;
};

/******************************************************************************
 * ROI_Refiner 类 —— ROI 精修器
 *
 * 对应真实项目: rect_refiner.cpp
 *
 * 真实项目中的 RectRefiner:
 *   - 根据检测框 (或先验框 PriorBox) 在原始图像上裁剪 ROI 区域
 *   - 对 ROI 做图像处理 (灰度化、边缘检测、轮廓查找)
 *   - 精修边界框, 使其更贴合目标边缘
 *   - 返回 RefinedRect (精修后的框 + 4 个角点)
 *
 * 这里简化: 对检测框做微小的收缩/扩张, 并用 corner 检测的模拟评分。
 *
 * 参数说明:
 *   raw_box   — 检测器给的原始框 (或 α-β 滤波预测的先验框)
 *   shrink_pct — 收缩百分比 (默认 0.10 = 10%)
 *   expand_pct — 扩张百分比 (默认 0.05 = 5%)
 *
 * 返回: BBox (精修后的框)
 *   - 如果评分 > 0.5, 返回精修框
 *   - 否则返回 nullopt (精修失败, 不可靠)
 *
 * 例子:
 *   原始框: (100, 100) ~ (200, 200), 宽高各 100 像素
 *   收缩 10%: 各边向内收缩 10 像素 → (110, 110) ~ (190, 190)
 *            模拟了: 去掉边缘可能的背景区域
 ******************************************************************************/
class ROI_Refiner {
public:
    ROI_Refiner(float shrink_pct = 0.10f, float expand_pct = 0.05f)
        : shrink_pct_(shrink_pct), expand_pct_(expand_pct)
        , rng_(std::random_device{}()) {}

    // 精修边界框
    // 对应真实项目: RectRefiner::Refine()
    std::optional<BBox> refine(const BBox& raw_box) {
        float w = raw_box.width();
        float h = raw_box.height();

        // 收缩: 从边界向内缩
        float shrink_x = w * shrink_pct_ * 0.5f;  // 每边缩一半
        float shrink_y = h * shrink_pct_ * 0.5f;

        // 扩张: 从边界向外扩 (模拟检测到更完整的边缘)
        float expand_x = w * expand_pct_ * 0.5f;
        float expand_y = h * expand_pct_ * 0.5f;

        // 精修后的框 = 原始框 - 收缩 + 扩张
        // 净效果: 收缩 5% (因为 shrink 10% > expand 5%)
        BBox refined(
            raw_box.x1 + shrink_x - expand_x,
            raw_box.y1 + shrink_y - expand_y,
            raw_box.x2 - shrink_x + expand_x,
            raw_box.y2 - shrink_y + expand_y,
            raw_box.conf  // 保持原置信度
        );

        // 模拟精修评分: 0.0 ~ 1.0
        // 65% 概率评分 > 0.5 (精修成功)
        std::uniform_real_distribution<float> score_dist(0.0f, 1.0f);
        float score = score_dist(rng_);

        if (score < 0.35f) {
            // 精修失败 (比如 ROI 区域太模糊, 找不到边缘)
            return std::nullopt;
        }

        refined.conf *= score;  // 置信度乘以评分 (真实项目会综合评估)
        return refined;
    }

private:
    float shrink_pct_;  // 收缩百分比
    float expand_pct_;  // 扩张百分比
    std::mt19937 rng_;
};

/******************************************************************************
 * Tracker 类 —— α-β 滤波器跟踪器
 *
 * 对应真实项目: tracker.cpp (RectTracker)
 *
 * 复用练习 B 中的 α-β 滤波逻辑, 但做了以下改进:
 *   - 加入了 TrackSnapshot 概念 (对应真实项目的 TrackSnapshot 结构体)
 *   - 宽高也用 EMA (指数移动平均) 平滑
 *   - 直接用项目中的状态字符串 (不是 enum)
 *
 * 真实项目 tracker.cpp 核心逻辑回顾:
 *   1. ApplyMeasurement(): 收到检测, 用 α-β 更新位置和速度
 *   2. Predict(): 没收到检测, 用匀速模型预测
 *   3. MarkDetectMiss(): 标记检测丢失, 增加 misses 计数
 *
 * 状态: "SEARCHING" → "LOCKED" → "LOST" → (回到 SEARCHING 或 LOCKED)
 ******************************************************************************/
class Tracker {
public:
    // 对应真实项目构造函数: RectTracker(const AppConfig& config)
    Tracker(float alpha = 0.7f, float beta = 0.3f,
            float box_alpha = 0.3f, int max_misses = 3)
        : alpha_(alpha), beta_(beta), box_alpha_(box_alpha)
        , max_misses_(max_misses)
        , initialized_(false)
        , x_(0), y_(0), w_(80), h_(80)
        , vx_(0), vy_(0)
        , misses_(0), hit_count_(0)
    {}

    // ===== 更新: 收到精修后的框 =====
    // 对应真实项目: RectTracker::Update(const RefinedRect& ...)
    void update(const BBox& box, double dt) {
        if (!initialized_ || misses_ > max_misses_) {
            // 首次初始化, 或跟踪丢失太久后重新初始化
            x_ = box.center_x();
            y_ = box.center_y();
            w_ = box.width();
            h_ = box.height();
            vx_ = 0.0f;
            vy_ = 0.0f;
            initialized_ = true;
        } else {
            // α-β 滤波 (和练习 B 完全一样)
            float px = x_ + vx_ * static_cast<float>(dt);  // 预测位置
            float py = y_ + vy_ * static_cast<float>(dt);
            float rx = box.center_x() - px;  // 残差
            float ry = box.center_y() - py;

            x_ = px + alpha_ * rx;   // 位置平滑
            y_ = py + alpha_ * ry;

            float dt_safe = static_cast<float>(std::max(1e-3, dt));
            vx_ += beta_ * rx / dt_safe;   // 速度平滑
            vy_ += beta_ * ry / dt_safe;

            // 宽高平滑: 指数移动平均 (EMA)
            // w_new = (1 - box_alpha) * w_old + box_alpha * w_measured
            // box_alpha = 0.3 → 新测量值占 30%, 旧值占 70%
            w_ = (1.0f - box_alpha_) * w_ + box_alpha_ * box.width();
            h_ = (1.0f - box_alpha_) * h_ + box_alpha_ * box.height();
        }

        // 重置丢失计数
        misses_ = 0;
        hit_count_++;
    }

    // ===== 标记本帧没有检测结果 =====
    // 对应真实项目: RectTracker::MarkDetectMiss()
    void mark_miss() {
        if (!initialized_) return;
        misses_++;
    }

    // ===== 纯预测 (用匀速模型) =====
    // 对应真实项目: RectTracker::Predict()
    void predict(double dt) {
        if (!initialized_) return;
        x_ += vx_ * static_cast<float>(dt);
        y_ += vy_ * static_cast<float>(dt);
    }

    // ===== 获取当前状态 (和真实项目一致) =====
    std::string state() const {
        if (!initialized_ || misses_ > max_misses_) return "SEARCHING";
        if (misses_ > 0) return "LOST";
        return "LOCKED";
    }

    // ===== 获取当前跟踪快照 =====
    // 对应真实项目: RectTracker::Snapshot() → TrackSnapshot
    struct Snapshot {
        bool   active;
        BBox   box;
        float  center_x, center_y;
        float  conf;
        int    misses;
        std::string state;
        std::string source;  // 数据来源: "ai" / "predict" / "none"
    };

    Snapshot snapshot(float conf = 1.0f, const std::string& source = "none") const {
        Snapshot s;
        s.active = initialized_ && misses_ <= max_misses_;
        s.misses = misses_;
        s.state  = state();
        s.source = s.active ? source : "none";
        s.conf   = (s.active ? conf : 0.0f);

        if (s.active) {
            // 从中心点反推边界框
            s.box       = BBox(x_ - w_ * 0.5f, y_ - h_ * 0.5f,
                               x_ + w_ * 0.5f, y_ + h_ * 0.5f, conf);
            s.center_x  = x_;
            s.center_y  = y_;
        }
        return s;
    }

    // 获取预测框 (PriorBox) — 给检测器当 ROI 提示
    // 对应真实项目: RectTracker::PriorBox()
    std::optional<BBox> prior_box() const {
        if (!initialized_ || misses_ > max_misses_) return std::nullopt;
        return BBox(x_ - w_ * 0.5f, y_ - h_ * 0.5f,
                    x_ + w_ * 0.5f, y_ + h_ * 0.5f, 0.5f);
    }

    // 统计
    int hit_count()  const { return hit_count_; }
    int miss_count() const { return misses_; }

private:
    float alpha_, beta_, box_alpha_;
    int max_misses_;
    bool initialized_;
    float x_, y_, w_, h_;   // 跟踪状态
    float vx_, vy_;         // 速度
    int misses_;             // 连续丢失帧数
    int hit_count_;          // 成功跟踪帧数 (统计用)
};

/******************************************************************************
 * OutputWriter 类 —— 输出跟踪状态
 *
 * 对应真实项目: aim_writer.cpp (AimWriter)
 *
 * 真实项目 AimWriter:
 *   - 将跟踪状态格式化为 $AIM,... 字符串
 *   - 通过串口 (/dev/ttyS0) 发给下位机 (STM32 单片机)
 *   - 如果没有串口, 就输出到 stdout
 *
 * $AIM 格式:
 *   $AIM,<found>,<state>,<cx>,<cy>,<dx>,<dy>,<conf>,<source>
 *
 *   字段说明:
 *     found  — 是否找到目标: 1=找到, 0=没找到
 *     state  — 状态: "LOCKED", "LOST", "SEARCHING"
 *     cx, cy — 目标中心点坐标 (像素)
 *     dx, dy — 目标偏离画面中心的偏移量 (像素)
 *     conf   — 置信度
 *     source — 数据来源: "ai"=AI检测, "predict"=预测, "none"=无
 *
 * 这里简化: 直接输出到终端 (stdout)
 ******************************************************************************/
class OutputWriter {
public:
    OutputWriter(double interval = 1.0)  // 输出间隔 (秒)
        : interval_(interval), last_output_(0.0) {}

    // 发送跟踪快照
    // 对应真实项目: AimWriter::Send()
    void send(const Tracker::Snapshot& snapshot, int frame_w, int frame_h) {
        double now = get_time();

        // 控制输出频率 (避免刷屏)
        if (now - last_output_ < interval_) return;
        last_output_ = now;

        // 格式化为 $AIM 字符串
        std::string aim = format_aim(snapshot, frame_w, frame_h);

        // 输出到终端 (真实项目输出到串口)
        std::cout << "  $AIM_OUTPUT: " << aim << std::endl;
    }

private:
    // 格式化 $AIM 字符串
    // 对应真实项目: AimWriter::FormatAim()
    std::string format_aim(const Tracker::Snapshot& s, int frame_w, int frame_h) {
        // 使用 C 风格的 snprintf 格式化 (和真实项目一样)
        // 好处: 性能极高, 适合嵌入式和实时系统
        char buffer[256];
        std::snprintf(buffer, sizeof(buffer),
                      "$AIM,%d,%s,%.1f,%.1f,%.1f,%.1f,%.3f,%s",
                      s.active ? 1 : 0,           // found
                      s.state.c_str(),            // state
                      s.center_x, s.center_y,     // cx, cy
                      s.center_x - frame_w * 0.5f, // dx (偏离中心)
                      s.center_y - frame_h * 0.5f, // dy (偏离中心)
                      s.conf,                     // confidence
                      s.source.c_str());          // data source
        return std::string(buffer);
    }

    // 获取当前时间 (秒)
    static double get_time() {
        using Clock = std::chrono::steady_clock;
        return std::chrono::duration<double>(
            Clock::now().time_since_epoch()).count();
    }

    double interval_;      // 输出间隔
    double last_output_;   // 上次输出时间
};

/******************************************************************************
 * Pipeline 类 —— 视觉管线编排器
 *
 * 对应真实项目: main.cpp 的 main() 函数主循环
 *
 * Pipeline 把所有组件串起来, 执行每一帧的处理流程:
 *
 *   for 每一帧:
 *     1. 从 CameraSource 读取最新帧
 *     2. 调用 Detector 检测目标
 *     3. 调用 ROI_Refiner 精修检测框
 *     4. 调用 Tracker 更新跟踪状态
 *     5. 调用 OutputWriter 输出结果
 *     6. 收集统计信息
 *
 * 这个类展示了面向对象设计中的 "组合" (Composition) 模式:
 *   Pipeline "拥有" 所有组件, 通过成员变量持有它们。
 ******************************************************************************/
class Pipeline {
public:
    // 构造函数: 创建所有组件
    // 参数是模拟画面的尺寸
    Pipeline(int frame_w = 640, int frame_h = 480)
        : camera_(frame_w, frame_h)
        , detector_()
        , refiner_(0.10f, 0.05f)
        , tracker_(0.7f, 0.3f, 0.3f, 3)
        , writer_(1.0)       // 每秒输出一次
        , frame_w_(frame_w)
        , frame_h_(frame_h)
        , running_(false)
    {
        std::cout << "[Pipeline] 初始化完成, 画面尺寸: "
                  << frame_w_ << "x" << frame_h_ << std::endl;
    }

    // 运行管线 (指定持续时间, 秒)
    void run(double duration_seconds = 3.0) {
        std::cout << "[Pipeline] 开始运行, 持续 " << duration_seconds << " 秒"
                  << std::endl;
        std::cout << std::endl;

        running_ = true;

        // 启动相机采集线程
        camera_.start();

        // 统计变量
        int total_frames = 0;      // 总帧数
        int detect_hits = 0;       // 检测命中次数
        int detect_misses = 0;     // 检测丢失次数
        int refine_ok = 0;         // 精修成功次数
        int refine_fail = 0;       // 精修失败次数
        double max_latency = 0.0;  // 最大处理延迟

        auto start_time = std::chrono::steady_clock::now();
        auto last_stats_time = start_time;

        // 主循环 — 对应真实项目 main.cpp 里的 while(true) 循环
        while (running_) {
            // 检查是否超时
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - start_time).count();
            if (elapsed >= duration_seconds) {
                running_ = false;
                break;
            }

            // ===== 步骤1: 读取最新帧 =====
            auto frame_opt = camera_.readLatest();
            if (!frame_opt.has_value()) {
                // 没有新帧 → 短暂休眠, 避免 busy-wait
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            Frame& frame = frame_opt.value();
            total_frames++;

            auto process_start = std::chrono::steady_clock::now();

            std::cout << "--- " << "Frame #" << frame.frame_id
                      << " (t=" << std::fixed << std::setprecision(1)
                      << frame.timestamp << "s) ---" << std::endl;

            // ===== 步骤2: AI 检测 =====
            // 模拟: 用 "真实位置" + 噪声 来生成检测结果
            float true_x = camera_.true_target_x();
            float true_y = camera_.true_target_y();
            auto detection = detector_.detect(frame, true_x, true_y);

            std::string data_source = "none";  // 跟踪数据来源
            bool detected_this_frame = false;

            if (detection.has_value()) {
                detect_hits++;
                detected_this_frame = true;
                std::cout << "  [Detector] 检测到目标: ";
                detection->print();
                std::cout << std::endl;
            } else {
                detect_misses++;
                std::cout << "  [Detector] 未检测到目标 (漏检)" << std::endl;
            }

            // ===== 步骤3: 获取先验框 (PriorBox) =====
            // 即使检测失败, tracker 也可以提供预测的 PriorBox
            auto prior = tracker_.prior_box();

            // 决定用哪个框做精修: 优先用检测框, 其次用 PriorBox
            std::optional<BBox> refine_input;
            if (detection.has_value()) {
                refine_input = detection;
                data_source = "ai";
            } else if (prior.has_value()) {
                refine_input = prior;
                data_source = "predict";
            }

            // ===== 步骤4: ROI 精修 =====
            std::optional<BBox> refined;
            if (refine_input.has_value()) {
                refined = refiner_.refine(*refine_input);
                if (refined.has_value()) {
                    refine_ok++;
                    std::cout << "  [Refiner] 精修成功, 评分="
                              << refined->conf << std::endl;
                } else {
                    refine_fail++;
                    std::cout << "  [Refiner] 精修失败 (ROI质量不足)" << std::endl;
                }
            }

            // ===== 步骤5: 跟踪器更新 =====
            // 真实项目 main.cpp 的优先顺序:
            //   1) 精修成功 → 用精修框更新 tracker
            //   2) 有检测结果但精修失败 → 用原始检测框更新
            //   3) 都没有 → 标记为丢失

            double dt = 1.0 / 5.0;  // 帧间隔 (5fps = 0.2s)

            if (refined.has_value()) {
                tracker_.update(*refined, dt);
                std::cout << "  [Tracker] 更新 (来源=" << data_source
                          << "), 状态=" << tracker_.state() << std::endl;
            } else if (detection.has_value()) {
                // 有检测结果但精修失败 → 用原始检测框
                tracker_.update(*detection, dt);
                std::cout << "  [Tracker] 更新 (来源=raw_ai), 状态="
                          << tracker_.state() << std::endl;
            } else {
                // 没有检测结果 → 标记丢失 + 预测
                tracker_.mark_miss();
                tracker_.predict(dt);
                std::cout << "  [Tracker] 丢失/预测, misses="
                          << tracker_.miss_count()
                          << ", 状态=" << tracker_.state() << std::endl;
            }

            // ===== 步骤6: 输出 =====
            // 取跟踪快照
            auto snapshot = tracker_.snapshot(
                detection.has_value() ? detection->conf : 0.0f,
                data_source
            );

            // 发送到输出
            writer_.send(snapshot, frame_w_, frame_h_);

            // ===== 计算处理延迟 =====
            auto process_end = std::chrono::steady_clock::now();
            double latency = std::chrono::duration<double>(
                process_end - process_start).count() * 1000.0;  // 转为毫秒
            if (latency > max_latency) max_latency = latency;

            std::cout << "  [延迟] " << latency << "ms" << std::endl;
            std::cout << std::endl;

            // ===== 定期输出统计 (模拟真实项目的 stats 输出) =====
            // 每 2 秒输出一次统计
            if (elapsed - last_stats_output_ >= 2.0) {
                print_stats(total_frames, detect_hits, detect_misses,
                            refine_ok, refine_fail, elapsed, max_latency);
                last_stats_output_ = elapsed;
            }
        }

        // 最后的统计
        auto end_time = std::chrono::steady_clock::now();
        double total_elapsed = std::chrono::duration<double>(
            end_time - start_time).count();

        // 停止相机采集线程
        camera_.stop();

        // 输出最终统计
        std::cout << std::endl;
        std::cout << "==================================================" << std::endl;
        std::cout << "  管线运行结束, 最终统计" << std::endl;
        std::cout << "==================================================" << std::endl;
        print_stats(total_frames, detect_hits, detect_misses,
                    refine_ok, refine_fail, total_elapsed, max_latency);
    }

private:
    // 打印统计信息 (模仿真实项目的 stats 输出格式)
    void print_stats(int total_frames, int detect_hits, int detect_misses,
                     int refine_ok, int refine_fail,
                     double elapsed, double max_latency) {
        // 真实项目的 stats 输出格式 (main.cpp 第 192-203 行):
        //   stats frame_fps=XX.X infer_fps=XX.X pre=X.Xms rknn=X.Xms ...
        //
        // 我们的简化版:

        double frame_fps = elapsed > 0.0 ? total_frames / elapsed : 0.0;

        // 使用 ostringstream 构造统计字符串
        // ostringstream 是 C++ 的字符串流, 类似于 sprintf 但是类型安全
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(1);

        ss << "stats"
           << " frame_fps=" << frame_fps
           << " total_frames=" << total_frames
           << " detect_hits=" << detect_hits
           << " detect_misses=" << detect_misses
           << " detect_rate=" << std::setprecision(0)
           << (total_frames > 0 ? detect_hits * 100.0 / total_frames : 0) << "%"
           << std::setprecision(1)
           << " refine_ok=" << refine_ok
           << " refine_fail=" << refine_fail
           << " max_latency=" << max_latency << "ms"
           << " tracker_hits=" << tracker_.hit_count()
           << " tracker_state=" << tracker_.state();

        std::cout << "  " << ss.str() << std::endl;
    }

    // ===== 管线组件 (组合 — Composition) =====
    CameraSource  camera_;
    Detector      detector_;
    ROI_Refiner   refiner_;
    Tracker       tracker_;
    OutputWriter  writer_;

    // ===== 配置 =====
    int frame_w_, frame_h_;
    std::atomic<bool> running_;
    double last_stats_output_ = -10.0;  // 上次输出统计的时间
};

/******************************************************************************
 * main 函数
 ******************************************************************************/
int main() {
    std::cout << std::fixed << std::setprecision(1);

    std::cout << "==================================================" << std::endl;
    std::cout << "  练习 D: 微型视觉管线模拟" << std::endl;
    std::cout << "==================================================" << std::endl;
    std::cout << std::endl;
    std::cout << "组件映射:" << std::endl;
    std::cout << "  CameraSource  → camera_source.cpp" << std::endl;
    std::cout << "  Detector      → rknn_detector.cpp/hpp" << std::endl;
    std::cout << "  ROI_Refiner   → rect_refiner.cpp" << std::endl;
    std::cout << "  Tracker       → tracker.cpp" << std::endl;
    std::cout << "  OutputWriter  → aim_writer.cpp" << std::endl;
    std::cout << "  Pipeline      → main.cpp" << std::endl;
    std::cout << std::endl;

    // 创建管线: 640x480 模拟画面
    Pipeline pipeline(640, 480);

    // 运行 4 秒 (比要求的 3 秒多一点, 方便观察完整周期)
    pipeline.run(3.0);

    std::cout << std::endl;
    std::cout << "==================================================" << std::endl;
    std::cout << "  练习 D 完成!" << std::endl;
    std::cout << "==================================================" << std::endl;
    std::cout << std::endl;
    std::cout << "你在这个练习中用到了前面的全部知识:" << std::endl;
    std::cout << "  练习 A: BBox 结构体 + IOU + vector" << std::endl;
    std::cout << "  练习 B: Tracker 类 + α-β 滤波 + 状态机" << std::endl;
    std::cout << "  练习 C: CameraSource 线程 + mutex + atomic + optional" << std::endl;
    std::cout << std::endl;
    std::cout << "新增的知识点:" << std::endl;
    std::cout << "  - 类组合 (Composition): Pipeline 包含所有组件" << std::endl;
    std::cout << "  - 管线模式 (Pipeline Pattern): 帧 → 检测 → 精修 → 跟踪 → 输出" << std::endl;
    std::cout << "  - std::ostringstream: 字符串格式化流" << std::endl;
    std::cout << "  - std::snprintf: C 风格高性能格式化" << std::endl;
    std::cout << "  - std::random_device + std::mt19937: 高质量随机数" << std::endl;
    std::cout << "  - 数据驱动设计: Snapshot 结构体作为组件间通信载体" << std::endl;
    std::cout << std::endl;
    std::cout << "恭喜你完成了全部 4 个递进练习!" << std::endl;
    std::cout << "你现在已经理解了 RK3576 视觉跟踪项目的核心架构。" << std::endl;

    return 0;
}
