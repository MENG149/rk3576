/******************************************************************************
 * 练习 B: 模拟目标跟踪器 (α-β 滤波器 + 状态机)
 *
 * 这个练习对应真实项目中的文件:
 *   cpp_fast_targeting/src/tracker.cpp
 *   cpp_fast_targeting/include/rkfast/tracker.hpp
 *
 * 真实项目里 RectTracker 类使用 α-β (alpha-beta) 滤波器来平滑跟踪目标位置,
 * 同时维护 SEARCHING / LOCKED / LOST 三态状态机。
 *
 * α-β 滤波器是最简单的卡尔曼滤波(Kalman Filter)的简化版, 用于估计目标的
 * 位置和速度。只用两个参数 α 和 β 就能实现平滑跟踪。
 *
 * 本练习覆盖的知识点:
 *   - class (类) 与 public/private 访问控制
 *   - enum class (强类型枚举)
 *   - 构造函数与成员初始化列表
 *   - const 成员函数
 *   - 状态机 (state machine) 设计模式
 *   - α-β 滤波器的数学公式
 *   - static 成员变量 (类共享的变量)
 *
 * 编译: g++ -std=c++17 -o practice_b_tracker practice_b_tracker.cpp
 * 运行: ./practice_b_tracker
 ******************************************************************************/

#include <algorithm>  // std::max, std::min, std::clamp
#include <cmath>      // std::abs
#include <iomanip>    // std::fixed, std::setprecision
#include <iostream>   // std::cout, std::endl
#include <string>     // std::string

/******************************************************************************
 * TrackState 枚举 —— 跟踪器状态
 *
 * C++ 里推荐使用 enum class (强类型枚举), 而不是 C 的 enum。
 *
 * enum class 的优点:
 *   1. 不能隐式转换为 int (更安全, 不会意外赋值)
 *   2. 使用时必须写 TrackState::SEARCHING (不会与其他枚举冲突)
 *   3. 命名空间隔离 (两个不同枚举可以有同名的值)
 *
 * 对应真实项目 tracker.cpp 里的状态字符串:
 *   state_ = "SEARCHING" / "LOCKED" / "LOST"
 *
 * 状态转移规则:
 *   SEARCHING → LOCKED    : 第一次检测到目标
 *   LOCKED    → LOST      : 连续 N 帧没有检测结果
 *   LOST      → LOCKED    : 重新检测到目标 (misses 清零)
 *   LOST      → SEARCHING : 连续丢失超过 max_misses 帧, 彻底放弃
 ******************************************************************************/
enum class TrackState {
    SEARCHING,  // 搜索中: 还没找到目标, 或目标丢失太久
    LOCKED,     // 已锁定: 正在稳定跟踪目标
    LOST        // 丢失中: 暂时丢失, 但还在用预测位置继续跟踪
};

// 把 TrackState 转成可读的字符串, 用于打印
const char* state_to_string(TrackState state) {
    switch (state) {
        case TrackState::SEARCHING: return "SEARCHING";
        case TrackState::LOCKED:    return "LOCKED";
        case TrackState::LOST:      return "LOST";
        default:                    return "UNKNOWN";
    }
}

/******************************************************************************
 * Target 类 —— 被跟踪的目标
 *
 * 这个类封装了一个目标的全部状态: 位置、大小、速度, 以及 α-β 滤波逻辑。
 *
 * 对应真实项目 tracker.cpp 中 RectTracker 类的核心变量:
 *   x_, y_, w_, h_   — 位置和大小
 *   vx_, vy_         — 速度
 *
 * 真实项目里这些是 RectTracker 的私有成员, 这里把跟踪逻辑拆分到 Target 类里,
 * 让职责更清晰 (单一职责原则 — Single Responsibility Principle)。
 ******************************************************************************/
class Target {
public:
    // ===== 成员变量 (为了教学方便, 这里用 public, 正式项目应该用 private) =====
    float x, y;     // 目标中心点坐标 (像素)
    float w, h;     // 目标框的宽和高 (像素)
    float vx, vy;   // 目标的 x 方向速度和 y 方向速度 (像素/秒)
    int misses;     // 连续丢失的帧数 (没有检测结果的帧数)

    // ===== 构造函数 =====
    // 接收初始位置和大小, 速度初始化为 0
    // 参数类型都用 float, 避免隐式类型转换
    Target(float init_x, float init_y, float init_w, float init_h)
        : x(init_x), y(init_y)
        , w(init_w), h(init_h)
        , vx(0.0f), vy(0.0f)
        , misses(0) {
        // 初始化列表全部搞定, 函数体为空
    }

    // ===== α-β 滤波更新 =====
    //
    // 这是跟踪算法的核心。每当我们从检测器得到一个测量值 (measured_x, measured_y),
    // 就用 α-β 滤波器融合 "预测值" 和 "测量值"。
    //
    // α-β 滤波公式 (经典教科书公式):
    //
    //   第1步 — 预测 (根据上一帧状态, 匀速模型推到当前帧):
    //     predicted_x = x + vx * dt     (如果在上次更新时就做了预测, 这里用已有的 x 即可)
    //     predicted_y = y + vy * dt
    //
    //   第2步 — 计算残差 (测量值 - 预测值):
    //     residual_x = measured_x - predicted_x
    //     residual_y = measured_y - predicted_y
    //
    //   第3步 — 平滑更新 (α 用于位置, β 用于速度):
    //     x  = predicted_x + α * residual_x     ← 位置 = 预测 + α * 修正
    //     y  = predicted_y + α * residual_y
    //     vx = vx          + β * residual_x / dt ← 速度 = 旧速度 + β * 修正 / 时间
    //     vy = vy          + β * residual_y / dt
    //
    // 通俗理解:
    //   α 控制 "你有多相信这次的测量值"
    //     α = 0.0 → 完全不相信测量, 只用预测 (非常平滑, 但滞后)
    //     α = 1.0 → 完全相信测量, 忽略预测 (紧跟目标, 但很抖动)
    //     α = 0.7 → 70% 相信测量 (常见折中值)
    //
    //   β 控制 "你根据残差调整速度的激进程度"
    //     β 通常比 α 小, 一般取 β = α² / (2 - α) 是一种经验公式
    //
    // 对应真实项目 tracker.cpp 里的 ApplyMeasurement():
    //   第 95-104 行就是 α-β 滤波的核心计算
    //
    // 参数:
    //   measured_x, measured_y  — 检测器给的位置 (测量值)
    //   alpha                   — 位置平滑系数 (0.0~1.0)
    //   beta                    — 速度平滑系数 (0.0~1.0)
    //   dt                      — 距离上一帧的时间间隔 (秒)
    void update(float measured_x, float measured_y,
                float alpha, float beta, float dt) {
        // 第1步: 用匀速模型预测当前位置
        // 注意: 如果这是第 0 帧, dt = 0, predict() 不会移动目标
        predict(dt);

        // 第2步: 计算残差 (测量值 - 预测值)
        float residual_x = measured_x - x;
        float residual_y = measured_y - y;

        // 第3步: 用 α 平滑位置
        x += alpha * residual_x;
        y += alpha * residual_y;

        // 第4步: 用 β 平滑速度
        // 速度公式: v_new = v_old + β * residual / dt
        // 注意: 当 dt 非常小时 (如 <= 1e-6), 残差/dt 会非常大, 导致速度爆炸
        // 所以要用 std::max 保护, 确保 dt 至少为 1e-6 秒
        float dt_safe = std::max(1e-6f, dt);
        vx += beta * residual_x / dt_safe;
        vy += beta * residual_y / dt_safe;

        // 更新成功 → 清零连续丢失计数
        misses = 0;

        // α-β 滤波的直观理解:
        //
        // 假设 α=0.7, β=0.3, dt=0.05 (20fps):
        //   - 预测位置 100, 测量位置 120 → 残差 = 20
        //   - 新位置 = 100 + 0.7*20 = 114  (向测量值移动了70%)
        //   - 新速度调整: 0.3*20/0.05 = 120 像素/秒 (因为残差大, 加速明显)
        //
        // 下一帧如果残差变小, 说明速度估计更准了, α-β 会逐渐收敛
    }

    // ===== 匀速预测 =====
    //
    // 在没有检测结果时, 用匀速模型 "猜" 目标的下一个位置。
    // 这模拟了牛顿第一定律: 物体在不受外力时保持匀速直线运动。
    //
    // x(t + dt) = x(t) + vx * dt
    // y(t + dt) = y(t) + vy * dt
    //
    // 对应真实项目 tracker.cpp 里的 Predict() 第 29-30 行
    //
    // 参数 dt: 距离上次更新的时间 (秒)
    //   - dt 越大, 预测的不确定性越大
    //   - 真实项目中用 std::clamp(dt, 1e-3, 0.25) 限制范围, 防止 dt 过大或过小
    void predict(float dt) {
        x += vx * dt;
        y += vy * dt;
        // 注意: 连续预测会让误差累积, 所以 misses 计数很重要
        // 另外, 真实项目还有边界约束 (Clamp), 防止预测到画面外
    }
};

/******************************************************************************
 * Tracker 类 —— 目标跟踪器
 *
 * 这个类管理一个 Target 和跟踪状态机, 协调 α-β 滤波的调用。
 *
 * 对应真实项目 tracker.cpp 里的 RectTracker 类:
 *   - ApplyMeasurement() → 我们封装为 update() + 状态转移
 *   - MarkDetectMiss()    → 我们封装为 no_detection() + 状态转移
 *   - Predict()           → 我们对应 predict()
 *
 * 状态转移逻辑 (参考 RectTracker 的行为):
 *
 *         ┌─────────────────────────────────┐
 *         │         SEARCHING                │
 *         │   (等待首次检测)                  │
 *         └──────┬──────────────────────────┘
 *                │ 检测到目标
 *                ▼
 *         ┌─────────────────────────────────┐
 *         │         LOCKED                  │
 *         │   (稳定跟踪中)                    │
 *         └──────┬──────────────────────────┘
 *                │ 连续丢帧 > max_misses
 *                ▼
 *         ┌─────────────────────────────────┐
 *         │         LOST                    │
 *         │   (暂丢, 继续预测)                │
 *         └──────┬──────────────────────────┘
 *         LOST + 重新检测到 → LOCKED
 *         LOST + 丢帧太多    → SEARCHING (重新开始)
 ******************************************************************************/
class Tracker {
public:
    // ===== 构造函数 =====
    // 传入初始目标位置和 α-β 参数
    Tracker(float init_x, float init_y, float init_w, float init_h,
            float alpha = 0.7f, float beta = 0.3f, int max_misses = 3)
        : target_(init_x, init_y, init_w, init_h)
        , state_(TrackState::SEARCHING)   // 初始状态: 搜索中
        , alpha_(alpha)
        , beta_(beta)
        , max_misses_(max_misses)
        , frame_count_(0)                 // static 变量在类外初始化
        , lock_count_(0)
        , lost_count_(0) {
        // 构造完成, 什么都不做
    }

    // ===== 收到检测结果时调用 =====
    //
    // 当检测器给出了一个目标位置, 调用这个方法:
    //   - 如果之前是 SEARCHING → 变成 LOCKED (首次锁定)
    //   - 如果之前是 LOST     → 变成 LOCKED (重新锁定)
    //   - 如果之前是 LOCKED   → 保持 LOCKED (继续跟踪)
    //
    // 然后调用 target_.update() 做 α-β 滤波
    void detected(float measured_x, float measured_y, float dt) {
        TrackState old_state = state_;

        // 状态转移: 不管之前是什么状态, 检测到目标就变 LOCKED
        if (state_ != TrackState::LOCKED) {
            if (state_ == TrackState::SEARCHING) {
                lock_count_++;
            }
            state_ = TrackState::LOCKED;
        }

        // 调用 Target 的 α-β 更新
        target_.update(measured_x, measured_y, alpha_, beta_, dt);

        // 打印状态变化
        if (old_state != state_) {
            std::cout << "  [状态变化] "
                      << state_to_string(old_state) << " → "
                      << state_to_string(state_) << std::endl;
        }
    }

    // ===== 本帧没有检测结果时调用 =====
    //
    // 检测器没找到目标, 追踪器需要自己 "猜" 目标位置 (匀速预测)。
    //
    // 状态转移:
    //   - SEARCHING → 保持 SEARCHING (我本来就不知道目标在哪)
    //   - LOCKED    → misses+1, 如果 >= max_misses → LOST
    //   - LOST      → misses+1, 如果 >= max_misses*2 → SEARCHING (彻底放弃)
    //
    // 对应真实项目 tracker.cpp 里的 MarkDetectMiss()
    void no_detection(float dt) {
        TrackState old_state = state_;

        if (state_ == TrackState::SEARCHING) {
            // 搜索中没检测到 → 啥也不做
            return;
        }

        // 增加丢失计数
        target_.misses++;

        // 状态转移逻辑
        if (state_ == TrackState::LOCKED && target_.misses >= max_misses_) {
            state_ = TrackState::LOST;
            lost_count_++;
        } else if (state_ == TrackState::LOST && target_.misses >= max_misses_ * 2) {
            // 丢太久了, 彻底放弃, 回到搜索状态
            state_ = TrackState::SEARCHING;
        }

        // 无论状态如何变化, 都用匀速模型预测下一步位置
        target_.predict(dt);

        if (old_state != state_) {
            std::cout << "  [状态变化] "
                      << state_to_string(old_state) << " → "
                      << state_to_string(state_) << std::endl;
        }
    }

    // ===== 纯预测 (不改变 misses 计数) =====
    // 对应真实项目 tracker.cpp 里的 Predict()
    // 当本帧既没有检测结果也不是 "没检测到" (比如检测器忙, 还没返回结果)
    // 调用这个方法用匀速模型推进目标位置
    void predict_only(float dt) {
        target_.predict(dt);
    }

    // ===== 获取当前状态 =====
    TrackState state() const { return state_; }

    // ===== 获取目标信息 (只读) =====
    // 返回 const 引用, 避免拷贝
    const Target& target() const { return target_; }

    // ===== 打印跟踪状态 =====
    void print_status() const {
        std::cout << "  [Tracker] 状态=" << state_to_string(state_)
                  << " | 位置=(" << target_.x << ", " << target_.y << ")"
                  << " | 速度=(" << target_.vx << ", " << target_.vy << ")"
                  << " | misses=" << target_.misses
                  << " | 锁定次数=" << lock_count_
                  << " | 丢失次数=" << lost_count_
                  << std::endl;
    }

private:
    Target target_;               // 被跟踪的目标 (组合 — Composition)
    TrackState state_;            // 当前状态
    float alpha_;                 // 位置平滑系数
    float beta_;                  // 速度平滑系数
    int max_misses_;              // 最大允许丢失帧数

    // ===== 统计计数器 (用于练习演示, 真实项目不需要) =====
    // static 成员变量: 这些变量属于 "整个类", 而不是 "某一个对象"
    // 所有 Tracker 对象共享同一份 lock_count_ 和 lost_count_
    //
    // 但在本练习中我们只有一个 Tracker, 所以这里的 static 主要是展示语法
    static int frame_count_;  // 总处理帧数 (static 需要在类外定义)
    int lock_count_;          // 锁定次数
    int lost_count_;          // 丢失次数
};

// static 成员变量的类外定义 (必须在 .cpp 文件或全局作用域中定义)
// static 成员变量不会在类内分配内存, 必须在这里分配
int Tracker::frame_count_ = 0;


/******************************************************************************
 * main 函数 —— 模拟完整的跟踪流程
 *
 * 模拟场景:
 *   1. 搜索阶段: 2 帧没有目标 (假装在扫描画面)
 *   2. 检测到目标: 锁定并开始跟踪
 *   3. 稳定跟踪: 跟随目标移动 10 帧 (α-β 滤波在起作用)
 *   4. 故意丢失: 3 帧不更新 → 状态变为 LOST
 *   5. 恢复检测: 重新锁定 → LOCKED
 *
 * 整个过程中, 每帧都打印状态, 方便观察 α-β 滤波效果和状态转移。
 ******************************************************************************/
int main() {
    // 设置浮点数输出格式: 保留 1 位小数
    std::cout << std::fixed << std::setprecision(1);

    std::cout << "==================================================" << std::endl;
    std::cout << "  练习 B: 模拟目标跟踪器 (α-β 滤波器 + 状态机)" << std::endl;
    std::cout << "==================================================" << std::endl;
    std::cout << std::endl;

    // ===== 创建跟踪器 =====
    // 初始位置: 画面中心 (320, 240), 目标大小 80x80
    // α=0.7 (70%相信测量), β=0.3 (中等速度平滑)
    // max_misses=3 (连续丢 3 帧进入 LOST, 丢 6 帧进入 SEARCHING)
    Tracker tracker(320.0f, 240.0f, 80.0f, 80.0f, 0.7f, 0.3f, 3);

    // 模拟帧率: 30fps → 每帧间隔约 0.033 秒
    float dt = 1.0f / 30.0f;  // 约 0.033 秒

    // dt 在真实项目中是动态计算的: dt = current_time - last_time
    // 这里为了简化, 固定为恒定的帧间隔

    // 模拟目标真实运动: 从 (320, 240) 向右下移动, 每秒移动 (50, 30) 像素
    float true_x = 320.0f;
    float true_y = 240.0f;
    float true_vx = 50.0f;   // 每秒向右移动 50 像素
    float true_vy = 30.0f;   // 每秒向下移动 30 像素

    int frame = 0;

    // ================================================================
    // 阶段1: 搜索阶段 (2 帧没有目标)
    // ================================================================
    std::cout << "----- 阶段1: 搜索阶段 (还没有检测到目标) -----" << std::endl;
    for (int i = 0; i < 2; ++i, ++frame) {
        std::cout << "帧 " << frame << ": 没有检测结果" << std::endl;
        tracker.no_detection(dt);
        tracker.print_status();

        // 目标继续真实移动 (但我们不知道)
        true_x += true_vx * dt;
        true_y += true_vy * dt;
    }
    std::cout << std::endl;

    // ================================================================
    // 阶段2: 首次检测到目标 → 锁定!
    // ================================================================
    std::cout << "----- 阶段2: 检测到目标! 锁定 ----" << std::endl;
    {
        true_x += true_vx * dt;
        true_y += true_vy * dt;
        ++frame;

        // 模拟检测结果: 真实位置 + 一些测量噪声 (±3 像素的随机误差)
        // 真实项目中噪声来自: 模型精度、光照变化、运动模糊等
        float noise_x = 3.0f;  // 模拟固定噪声 (实际是随机的)
        float noise_y = -2.0f;
        float measured_x = true_x + noise_x;
        float measured_y = true_y + noise_y;

        std::cout << "帧 " << frame << ": 检测到目标! "
                  << "真实位置=(" << true_x << "," << true_y << ") "
                  << "测量位置=(" << measured_x << "," << measured_y << ") "
                  << "(含噪声)" << std::endl;

        tracker.detected(measured_x, measured_y, dt);
        tracker.print_status();

        std::cout << "  [α-β效果] 跟踪器位置接近测量值, 速度从0开始建立"
                  << std::endl;
    }
    std::cout << std::endl;

    // ================================================================
    // 阶段3: 稳定跟踪 10 帧 (目标匀速移动, 每帧都有测量值)
    // ================================================================
    std::cout << "----- 阶段3: 稳定跟踪 10 帧 -----" << std::endl;
    std::cout << "(观察 α-β 滤波如何平滑噪声测量值)" << std::endl;

    for (int i = 0; i < 10; ++i, ++frame) {
        // 目标真实位置更新
        true_x += true_vx * dt;
        true_y += true_vy * dt;

        // 模拟检测噪声: 有时偏左, 有时偏右, 模拟测量不稳定性
        // 使用简单的交替噪声来展示滤波效果
        float noise_x = (i % 2 == 0) ? 4.0f : -3.0f;  // 交替 ±3~4 像素
        float noise_y = (i % 3 == 0) ? -5.0f : 2.0f;   // 交替噪声
        float measured_x = true_x + noise_x;
        float measured_y = true_y + noise_y;

        float track_err = std::abs(tracker.target().x - true_x);

        std::cout << "帧 " << frame
                  << " | 真=(" << true_x << "," << true_y << ")"
                  << " | 测=(" << measured_x << "," << measured_y << ")"
                  << " | 跟=(" << tracker.target().x << "," << tracker.target().y << ")"
                  << " | 速=(" << tracker.target().vx << "," << tracker.target().vy << ")"
                  << " | 误差=" << track_err << "px"
                  << std::endl;

        tracker.detected(measured_x, measured_y, dt);

        // 观察 α-β 滤波效果:
        //   - 跟踪位置 (x,y) 比测量值更平滑, 不会跳来跳去
        //   - 速度 (vx, vy) 逐渐收敛到真实速度 (50, 30)
        //   - 跟踪误差始终比测量噪声小 (这就是滤波的作用)
    }
    std::cout << std::endl;

    // ================================================================
    // 阶段4: 故意丢失 3 帧 (模拟目标被遮挡或检测器误判)
    // ================================================================
    std::cout << "----- 阶段4: 故意不更新 3 帧 (模拟遮挡/检测失败) -----" << std::endl;
    std::cout << "(观察跟踪器如何用匀速模型预测, 以及状态变化)" << std::endl;

    for (int i = 0; i < 3; ++i, ++frame) {
        // 目标继续真实移动
        true_x += true_vx * dt;
        true_y += true_vy * dt;

        float predict_err = std::abs(tracker.target().x - true_x);

        std::cout << "帧 " << frame
                  << " | 真=(" << true_x << "," << true_y << ")"
                  << " | 预测位置=(" << tracker.target().x << "," << tracker.target().y << ")"
                  << " | 预测误差=" << predict_err << "px"
                  << " | misses=" << tracker.target().misses
                  << std::endl;

        tracker.no_detection(dt);

        // 观察:
        //   - misses 计数逐渐增加
        //   - 跟踪器用匀速模型预测位置 (在没有测量的情况下)
        //   - 预测误差会逐渐累积 (因为没有测量校正)
        //   - 当 misses >= 3 时, 状态从 LOCKED → LOST
    }
    std::cout << std::endl;

    // ================================================================
    // 阶段5: 丢失后再丢 3 帧 (观察 LOST → SEARCHING 转换)
    // ================================================================
    std::cout << "----- 阶段5: 继续丢失 3 帧 (LOST → SEARCHING) -----" << std::endl;
    std::cout << "(misses 累积到 max_misses*2 = 6 时, 彻底放弃搜索)" << std::endl;

    for (int i = 0; i < 3; ++i, ++frame) {
        true_x += true_vx * dt;
        true_y += true_vy * dt;

        std::cout << "帧 " << frame
                  << " | 预测位置=(" << tracker.target().x << "," << tracker.target().y << ")"
                  << " | misses=" << tracker.target().misses
                  << std::endl;

        tracker.no_detection(dt);

        // 当 misses 达到 6 (max_misses * 2), 状态变成 SEARCHING
        // 表示 "丢失太久, 放弃这个目标, 重新开始搜索"
    }
    std::cout << std::endl;

    // ================================================================
    // 阶段6: 重新检测到目标 → 重新锁定!
    // ================================================================
    std::cout << "----- 阶段6: 重新检测到目标! 重新锁定 -----" << std::endl;

    {
        true_x += true_vx * dt;
        true_y += true_vy * dt;
        ++frame;

        float measured_x = true_x + 2.0f;  // 带一点噪声
        float measured_y = true_y - 1.0f;

        std::cout << "帧 " << frame << ": 重新检测到目标!"
                  << " 测量=(" << measured_x << "," << measured_y << ")"
                  << std::endl;

        tracker.detected(measured_x, measured_y, dt);
        tracker.print_status();

        std::cout << "  [注意] 跟踪器会在测量位置附近重建跟踪, 速度重新从0开始"
                  << std::endl;
    }
    std::cout << std::endl;

    // ================================================================
    // 阶段7: 重新跟踪几帧, 确认状态恢复
    // ================================================================
    std::cout << "----- 阶段7: 恢复后继续跟踪 3 帧 -----" << std::endl;

    for (int i = 0; i < 3; ++i, ++frame) {
        true_x += true_vx * dt;
        true_y += true_vy * dt;

        float measured_x = true_x + ((i == 1) ? -2.0f : 1.0f);
        float measured_y = true_y + ((i == 1) ? 3.0f : -1.0f);

        std::cout << "帧 " << frame
                  << " | 跟=(" << tracker.target().x << "," << tracker.target().y << ")"
                  << " | 速=(" << tracker.target().vx << "," << tracker.target().vy << ")"
                  << " | 状态=" << state_to_string(tracker.state())
                  << std::endl;

        tracker.detected(measured_x, measured_y, dt);
    }

    std::cout << std::endl;
    std::cout << "==================================================" << std::endl;
    std::cout << "  练习 B 完成" << std::endl;
    std::cout << "==================================================" << std::endl;
    std::cout << std::endl;
    std::cout << "你学会了:" << std::endl;
    std::cout << "  - class 类: public/private, 成员函数, 构造函数" << std::endl;
    std::cout << "  - enum class: 强类型枚举, 状态机模式" << std::endl;
    std::cout << "  - α-β 滤波器: 位置平滑 + 速度估计" << std::endl;
    std::cout << "  - 状态转移: SEARCHING → LOCKED → LOST → ..." << std::endl;
    std::cout << "  - 组合 (Composition): Tracker 包含 Target" << std::endl;
    std::cout << std::endl;
    std::cout << "对应真实项目: cpp_fast_targeting/src/tracker.cpp" << std::endl;
    std::cout << "下一步: 练习 C — 多线程生产者-消费者" << std::endl;

    return 0;
}
