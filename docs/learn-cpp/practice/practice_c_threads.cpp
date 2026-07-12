/******************************************************************************
 * 练习 C: 多线程生产者-消费者 (模拟项目三线程架构)
 *
 * 这个练习对应真实项目中的文件:
 *   cpp_fast_targeting/src/camera_source.cpp    (生产者线程: CaptureLoop)
 *   cpp_fast_targeting/include/rkfast/rknn_detector.hpp  (DetectorWorker)
 *
 * 真实项目的线程架构:
 *
 *   ┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
 *   │ Camera Thread   │────▶│ Detector Thread  │────▶│  Main Thread    │
 *   │ (生产者)         │     │ (消费者+生产者)    │     │ (最终消费者)     │
 *   │                 │     │                  │     │                 │
 *   │ 不断采集图像帧   │     │ 取出最新帧做推理   │     │ 取推理结果       │
 *   │ 放入共享buffer  │     │ 产生检测结果      │     │ 渲染+输出        │
 *   └─────────────────┘     └──────────────────┘     └─────────────────┘
 *
 * 本练习简化成两线程:
 *   - 主线程   (生产者): 模拟产生 5 帧图像
 *   - 推理线程 (消费者): 从共享 buffer 取帧, 模拟 1 秒处理时间
 *   - 主线程   (消费者): 取推理结果
 *
 * "丢旧帧" 策略:
 *   当消费者处理慢时, 生产者不等消费者完成, 新帧直接覆盖旧帧。
 *   这模拟真实项目中 "只处理最新帧" 的策略 — 旧帧的处理结果已经没用了。
 *
 * 本练习覆盖的知识点:
 *   - std::thread          (线程)
 *   - std::mutex            (互斥锁, 保护共享数据)
 *   - std::lock_guard       (RAII 风格的自动加锁/解锁)
 *   - std::optional<T>      (可能为 "空" 的值)
 *   - std::atomic<bool>     (原子布尔, 线程安全的标志)
 *   - std::this_thread::sleep_for (让线程休眠)
 *   - std::chrono           (时间库)
 *   - RAII 模式 (Resource Acquisition Is Initialization)
 *
 * 编译: g++ -std=c++17 -pthread -o practice_c_threads practice_c_threads.cpp
 * 运行: ./practice_c_threads
 ******************************************************************************/

#include <atomic>    // std::atomic<bool> — 原子操作, 不需要锁就能安全读写
#include <chrono>    // std::chrono::milliseconds, std::chrono::steady_clock
#include <cstdint>   // uint64_t — 无符号 64 位整数
#include <iostream>  // std::cout, std::endl
#include <mutex>     // std::mutex, std::lock_guard
#include <optional>  // std::optional<T> — 可能有值, 也可能为 "空"
#include <string>    // std::string
#include <thread>    // std::thread, std::this_thread::sleep_for

/******************************************************************************
 * FrameBuffer 类 —— 线程安全的帧缓冲区
 *
 * 对应真实项目: camera_source.cpp 里的 CaptureLoop + ReadLatest
 *
 * 在真实项目中, 相机线程不断采集帧, 存入一个受 mutex 保护的 cv::Mat。
 * 主线程随时读取最新的帧 (ReadLatest), 如果同一帧被读了多次也没关系。
 *
 * 这里的 FrameBuffer 原理相同:
 *   - produce(): 生产者把帧放进来 (覆盖旧帧)
 *   - consume(): 消费者取出最新帧 (通过序列号判断是否为新帧)
 *
 * 线程安全机制:
 *   - 所有对内部状态的访问都被 std::mutex 保护
 *   - std::lock_guard 在构造时自动加锁, 析构时自动解锁 (RAII)
 *   - 即使函数中间抛异常, lock_guard 也会自动解锁 (不会死锁)
 *
 * "丢旧帧" 的意义:
 *   如果生产者比消费者快 10 倍, 缓冲区里可能堆积很多帧。
 *   策略: 只保留最新帧 (覆盖旧帧), 消费者拿到的是最新帧, 旧帧被丢弃。
 *   为什么? 因为视觉跟踪需要最新的画面, 处理 500ms 前的旧画面没有意义。
 ******************************************************************************/
class FrameBuffer {
public:
    // 构造函数: 初始化序列号为 0
    FrameBuffer() : seq_(0), has_new_(false) {}

    // ===== 生产者接口: 放入一帧 =====
    //
    // 生产者 (如相机线程) 调用这个方法, 把新采集的帧放进来。
    // 如果缓冲区里已经有旧帧还没被消费, 旧帧会被直接覆盖 (丢旧帧策略)。
    //
    // 参数 frame_name: 帧的名字 (真实项目里是 cv::Mat 图像数据,
    //                  这里简化为字符串, 方便演示)
    void produce(const std::string& frame_name) {
        // std::lock_guard 是 RAII 风格的锁:
        //   - 构造时自动调用 mutex_.lock()
        //   - 析构时自动调用 mutex_.unlock()
        // 即使函数提前 return 或抛异常, lock_guard 也会自动解锁
        // 永远不要手动 lock()/unlock(), 容易出错 (忘了 unlock 就死锁了)
        std::lock_guard<std::mutex> lock(mutex_);

        // 分配新的序列号 (单调递增, 每帧唯一)
        ++seq_;

        // 覆盖旧帧 (丢旧帧策略的核心)
        latest_frame_ = frame_name;
        has_new_ = true;

        // lock_guard 在这里析构 → 自动解锁
        // 锁的范围 (critical section) 只有上面这几行, 非常短
        // 加锁的代码越短越好, 因为锁会阻塞其他线程
    }

    // ===== 消费者接口: 取出最新帧 =====
    //
    // 消费者 (如推理线程) 调用这个方法, 取出最新帧。
    //
    // 返回值: std::optional<std::string>
    //   - 如果有新帧 (序列号变了) → 返回帧名
    //   - 如果没有新帧 (序列号没变) → 返回 std::nullopt
    //
    // std::optional<T> 是 C++17 引入的, 用来表示 "可能没有值" 的情况。
    // 它比传统的方案更安全:
    //   - 传统方案: 返回空字符串 "" 表示 "没数据", 但 "" 本身也是一个合法的字符串
    //   - optional  : 语义明确 — has_value() 检查是否有数据, 不会歧义
    //
    // 真实项目大量使用 std::optional:
    //   std::optional<Detection>  — 可能有检测结果, 也可能没有
    //   std::optional<Box>        — 可能有先验框(PriorBox), 也可能没有
    std::optional<std::string> consume() {
        std::lock_guard<std::mutex> lock(mutex_);

        // 检查序列号: 只有序列号变化才说明有新帧
        if (has_new_ && last_consumed_seq_ != seq_) {
            // 记录本次消费的序列号 (避免重复消费同一帧)
            last_consumed_seq_ = seq_;
            has_new_ = false;
            return latest_frame_;  // optional 自动包装: "我有值"
        }

        // 没有新帧 → 返回 "空" optional
        return std::nullopt;
        // std::nullopt 是 C++17 提供的常量, 表示 optional 为空
    }

    // ===== 查询当前序列号 (只读, 不需要返回数据) =====
    uint64_t current_seq() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return seq_;
    }

private:
    // mutable 关键字:
    //   即使成员函数是 const (如 current_seq()), 也能修改 mutable 成员
    //   这对 mutex 是必要的 — const 函数里也需要加锁
    mutable std::mutex mutex_;  // 互斥锁, 保护下面所有成员

    std::string latest_frame_;   // 最新帧的数据 (真实项目是 cv::Mat)
    uint64_t seq_;               // 当前序列号 (生产者每放一帧 +1)
    uint64_t last_consumed_seq_; // 上次消费的序列号 (消费者用来判断是否新帧)
    bool has_new_;               // 是否有未被消费的新帧
};


/******************************************************************************
 * DetectorWorker 类 —— 模拟 AI 推理工作线程
 *
 * 对应真实项目: rknn_detector.hpp 里的 DetectorWorker
 *
 * 真实项目的 DetectorWorker:
 *   - 接收 cv::Mat 帧 → Submit()
 *   - 在工作线程里调用 RknnDetector::Infer() 做 NPU 推理
 *   - 推理完成后结果放在 latest_result_ 里
 *   - 主线程通过 LatestResult() 取结果
 *   - Busy() 检查推理线程是否正在忙
 *
 * 本练习的简化版:
 *   - 接收 std::string 帧名 → Submit()
 *   - 工作线程 sleep 1 秒模拟 AI 推理耗时 (真实项目推理 15~30ms, 这里放慢方便观察)
 *   - 推理完成后结���放在 latest_result_ 里
 *   - result() 取最新结果
 *
 * 线程安全机制:
 *   - submit() 和 result() 都用 mutex 保护
 *   - stop_flag_ 是 std::atomic<bool>, 不需要锁
 *   - run() 在工作线程中的 while 循环受 stop_flag_ 控制
 ******************************************************************************/
class DetectorWorker {
public:
    // 构造函数
    DetectorWorker()
        : stop_flag_(false)   // 原子变量的初始化
        , result_seq_(0)       // 结果序列号从 0 开始
    {}

    // 析构函数: 确保线程被停止
    // 如果忘记调用 stop(), 析构函数会自动处理
    ~DetectorWorker() {
        stop();
    }

    // ===== 启动工作线程 =====
    //
    // 创建并启动一个新线程, 运行 run() 函数。
    //
    // std::thread 的构造函数接收一个 "可调用对象" (函数、lambda、函数对象等),
    // 以及可选的参数。这里我们传 this 指针:
    //
    //   std::thread(&DetectorWorker::run, this);
    //
    // 含义: 在新线程中调用 this->run()
    //
    // .detach() 表示 "分离线程" — 线程独立运行, 不需要 join()。
    // 我们用 stop() + join() 来干净地停止线程, 所以这里用 detach() 也行,
    // 但为了教学我们保留 thread_ 句柄以便后续 join。
    void start() {
        stop_flag_.store(false);   // atomic 的 store, 线程安全
        thread_ = std::thread(&DetectorWorker::run, this);
        std::cout << "  [DetectorWorker] 推理线程已启动" << std::endl;
    }

    // ===== 停止工作线程 =====
    //
    // 1. 设置停止标志 (atomic, 其他线程立即可见)
    // 2. 等待线程结束 (join)
    //
    // join() 会阻塞当前线程, 直到被 join 的线程结束。
    // 这保证了在 stop() 返回后, 工作线程已经完全退出。
    void stop() {
        stop_flag_.store(true);  // atomic store — 写入对所有线程立即可见

        // joinable() 检查线程是否可 join (是否已启动且未 detach)
        if (thread_.joinable()) {
            std::cout << "  [DetectorWorker] 等待推理线程结束..." << std::endl;
            thread_.join();  // 阻塞等待线程结束
            std::cout << "  [DetectorWorker] 推理线程已停止" << std::endl;
        }
    }

    // ===== 提交推理任务 =====
    //
    // 生产者 (主线程) 调用, 把一帧提交给推理线程。
    //
    // 策略: 如果推理线程正在忙, 旧任务被覆盖 — 等价于 "丢旧帧"。
    //
    // 对应真实项目: DetectorWorker::Submit()
    void submit(const std::string& frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_frame_ = frame;
        pending_ = true;
        // 注意: 如果推理线程正在处理上一帧, 这里直接覆盖 pending_frame_
        // 推理线程完成当前任务后, 会处理这个新提交的帧
    }

    // ===== 获取最新推理结果 (非阻塞) =====
    //
    // 消费者 (主线程) 调用, 检查是否有新的推理结果。
    //
    // 返回 std::optional<std::string>:
    //   - 有新结果 → 返回结果字符串 (格式: "Detected: Frame_N")
    //   - 没有新结果 → 返回 nullopt
    //
    // 对应真实项目: DetectorWorker::LatestResult()
    std::optional<std::string> result() {
        std::lock_guard<std::mutex> lock(mutex_);

        // 通过序列号判断是否为新结果
        if (latest_result_.has_value() && last_read_seq_ != result_seq_) {
            last_read_seq_ = result_seq_;
            return latest_result_;
        }
        return std::nullopt;
    }

    // ===== 查询推理线程是否正在忙 =====
    //
    // 对应真实项目: DetectorWorker::Busy()
    bool busy() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return processing_;
    }

private:
    // ===== 工作线程主循环 =====
    //
    // 这个函数在独立线程中运行 (start() 创建的线程)。
    //
    // while 循环检查 stop_flag_ (atomic):
    //   - stop_flag_ 是 std::atomic<bool>, 读写都是线程安全的
    //   - 不需要锁, 因为 CPU 保证原子读/写操作的完整性
    //   - 但普通的 bool 不行: 一个线程写, 另一个线程读, 可能读到中间状态
    void run() {
        std::cout << "  [DetectorWorker::run] 推理线程开始运行" << std::endl;

        // .load() 是 atomic 的读取方法
        // 也可以用隐式转换: while (!stop_flag_) — 但 .load() 更明确
        while (!stop_flag_.load()) {
            // 取出待处理的帧 (加锁, 因为 pending_frame_ 非原子)
            std::string frame_to_process;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (pending_) {
                    frame_to_process = pending_frame_;
                    pending_ = false;
                    processing_ = true;  // 标记为 "处理中"
                }
            }  // 锁在这里释放 — 临界区结束

            if (!frame_to_process.empty()) {
                // 模拟 AI 推理耗时
                // 真实项目: 推理耗时 15-30ms (取决于模型和设备)
                // 这里用 1000ms 方便观察多线程行为
                std::cout << "  [推理] 开始处理: " << frame_to_process
                          << " (模拟 1 秒推理...)" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                // 生成推理结果
                std::string result = "Detected: " + frame_to_process;

                // 将结果存入共享变量
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    latest_result_ = result;
                    ++result_seq_;     // 递增结果序列号
                    processing_ = false; // 标记为 "空闲"
                }

                std::cout << "  [推理] 完成: " << result << std::endl;
            } else {
                // 没有待处理的任务 → 短暂休眠, 避免 busy-waiting
                // busy-waiting 会 100% 占满 CPU, 浪费电
                // 真实项目用条件变量 (std::condition_variable) 更高效
                // 这里用 sleep 简单演示
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        std::cout << "  [DetectorWorker::run] 推理线程退出" << std::endl;
    }

    // ===== 线程安全的共享状态 =====
    //
    // mutex 保护的范围: 所有非原子的成员变量
    //   为什么要保护? 如果两个线程同时写同一个变量, 结果是未定义的 (UB)
    //   即使一个写一个读, 也可能读到 "写了一半" 的脏数据
    //
    // atomic 不需要 mutex:
    //   std::atomic 保证读写操作的原子性, CPU 已经保证了

    mutable std::mutex mutex_;  // mutable 允许 const 函数里加锁

    // === 受 mutex 保护的成员 ===
    std::string pending_frame_;                // 待处理的帧
    bool pending_ = false;                     // 是否有待处理帧
    bool processing_ = false;                  // 是否正在处理
    std::optional<std::string> latest_result_; // 最新推理结果
    uint64_t result_seq_ = 0;                  // 结果序列号
    uint64_t last_read_seq_ = 0;               // 上次读取的序列号

    // === 原子变量 (不需要 mutex) ===
    std::atomic<bool> stop_flag_;  // 停止标志

    // === 线程句柄 ===
    std::thread thread_;  // 工作线程对象
};


/******************************************************************************
 * main 函数 —— 模拟生产者-消费者流程
 *
 * 模拟场景:
 *   主线程生产 5 帧 → 推理线程消费 → 主线程取推理结果
 *
 * 时序示意 (假设推理耗时 1s, 生产间隔 0.3s):
 *
 *   时间  主线程(生产者)          推理线程(消费者)
 *   ----  --------------------    --------------------
 *   0.0s  提交 Frame_0           开始处理 Frame_0 (1s)
 *   0.3s  提交 Frame_1           (忙, Frame_1 覆盖 pending)
 *   0.6s  提交 Frame_2           (忙, Frame_2 覆盖 pending)
 *   0.9s  提交 Frame_3           (忙, Frame_3 覆盖 pending)
 *   1.0s                         完成 Frame_0 → 结果 Ready
 *   1.0s  主线程取到 Frame_0 结果
 *   1.0s                         取到 Frame_3 (Frame_1,2 被丢了!)
 *   1.2s  提交 Frame_4           (忙)
 *   2.0s                         完成 Frame_3 → 结果 Ready
 *   ...
 *
 * 关键观察: Frame_1 和 Frame_2 被 "丢了" — 这就是真实项目中的丢帧策略。
 * 因为处理旧帧没有意义, 只需要最新帧的检测结果。
 ******************************************************************************/
int main() {
    std::cout << "==================================================" << std::endl;
    std::cout << "  练习 C: 多线程生产者-消费者" << std::endl;
    std::cout << "==================================================" << std::endl;
    std::cout << std::endl;

    std::cout << "架构说明:" << std::endl;
    std::cout << "  主线程 = 生产者 (产生模拟帧) + 消费者 (取推理结果)" << std::endl;
    std::cout << "  推理线程 = 消费者 (取帧处理) + 生产者 (产生检测结果)" << std::endl;
    std::cout << "  FrameBuffer 和 DetectorWorker 各自有独立的 mutex 保护" << std::endl;
    std::cout << std::endl;

    // ===== 创建组件 =====
    FrameBuffer frame_buffer;
    DetectorWorker detector;

    // ===== 启动推理线程 =====
    detector.start();

    // 等一小会儿让推理线程进入等待循环
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::cout << std::endl;

    // ================================================================
    // 阶段1: 主线程生产 5 帧
    // ================================================================
    std::cout << "----- 阶段1: 主线程生产 5 帧 -----" << std::endl;
    std::cout << "(推理线程耗时 1 秒, 主线程每 0.3 秒生产一帧)" << std::endl;
    std::cout << "(观察哪些帧被丢弃)" << std::endl;
    std::cout << std::endl;

    for (int i = 0; i < 5; ++i) {
        std::string frame_name = "Frame_" + std::to_string(i);

        // 步骤1: 模拟采集帧 (真实项目是相机捕获)
        std::cout << "[主线程] 采集帧: " << frame_name << std::endl;

        // 步骤2: 放入共享 Buffer
        frame_buffer.produce(frame_name);
        std::cout << "         放入 FrameBuffer (seq="
                  << frame_buffer.current_seq() << ")" << std::endl;

        // 步骤3: 提交给推理线程 (如果有旧任务, 直接覆盖)
        detector.submit(frame_name);
        std::cout << "         提交给推理线程 (busy="
                  << (detector.busy() ? "是" : "否") << ")" << std::endl;

        // 模拟帧间隔 (真实项目是 20~60fps, 这里用 0.3 秒方便观察)
        if (i < 4) {  // 最后一帧不需要再等了
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
    }

    std::cout << std::endl;

    // ================================================================
    // 阶段2: 主线程等待并获取推理结果
    // ================================================================
    std::cout << "----- 阶段2: 主线程取推理结果 -----" << std::endl;
    std::cout << "(非阻塞轮询, 直到所有结果都取出)" << std::endl;
    std::cout << std::endl;

    int results_received = 0;
    int max_wait_attempts = 20;  // 最多等 20 次 (每次 0.5 秒 = 最多等 10 秒)

    for (int attempt = 0; attempt < max_wait_attempts; ++attempt) {
        // 尝试取结果 (非阻塞)
        auto result = detector.result();

        if (result.has_value()) {
            // has_value() 检查 optional 是否有值
            results_received++;
            std::cout << "[主线程] 收到推理结果 #" << results_received
                      << ": " << result.value() << std::endl;
            // .value() 取出 optional 内的值
            // 注意: 如果 optional 为空, .value() 会抛异常
            // 所以一定要先 has_value() 再 .value()
        } else {
            std::cout << "[主线程] 轮询: 暂无结果 (busy="
                      << (detector.busy() ? "是" : "否") << ")" << std::endl;
        }

        // 从 FrameBuffer 消费帧 (演示消费者接口)
        auto frame = frame_buffer.consume();
        if (frame.has_value()) {
            std::cout << "         FrameBuffer 有新帧: " << frame.value()
                      << std::endl;
        }

        // 如果推理线程不忙了, 且没有更多帧要处理, 可以提前退出
        if (!detector.busy() && results_received >= 1) {
            // 再等一轮确认没有遗漏的结果
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            auto final_result = detector.result();
            if (final_result.has_value()) {
                results_received++;
                std::cout << "[主线程] 收到最终推理结果 #" << results_received
                          << ": " << final_result.value() << std::endl;
            }
            break;
        }

        // 轮询间隔
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << std::endl;
    std::cout << "统计: 生产了 5 帧, 收到 " << results_received
              << " 个推理结果" << std::endl;
    std::cout << "(由于处理慢, 中间帧被丢弃, 只处理了最新帧 — "
              << "这就是真实项目中的策略)" << std::endl;

    // ================================================================
    // 清理
    // ================================================================
    std::cout << std::endl;
    std::cout << "----- 停止推理线程 -----" << std::endl;
    detector.stop();

    std::cout << std::endl;
    std::cout << "==================================================" << std::endl;
    std::cout << "  练习 C 完成" << std::endl;
    std::cout << "==================================================" << std::endl;
    std::cout << std::endl;
    std::cout << "你学会了:" << std::endl;
    std::cout << "  - std::thread: 创建和管理线程" << std::endl;
    std::cout << "  - std::mutex + lock_guard: 线程同步" << std::endl;
    std::cout << "  - std::atomic<bool>: 无锁的线程安全标志" << std::endl;
    std::cout << "  - std::optional<T>: 安全地表示 '可能没有值'" << std::endl;
    std::cout << "  - 生产者-消费者模式: 丢旧帧策略" << std::endl;
    std::cout << "  - RAII: 利用析构函数自动清理资源" << std::endl;
    std::cout << std::endl;
    std::cout << "对应真实项目:" << std::endl;
    std::cout << "  camera_source.cpp  — CaptureLoop (FrameBuffer 的 produce)" << std::endl;
    std::cout << "  rknn_detector.hpp  — DetectorWorker (Submit/LatestResult)" << std::endl;
    std::cout << "下一步: 练习 D — 微型视觉管线" << std::endl;

    return 0;
}
