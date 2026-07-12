# 03 — STL 常用容器：你项目里用到的那些 `< >`

## 为什么需要这些东西

C 语言你用：
```c
char name[100] = "hello";     // 固定大小的字符数组
int nums[10];                 // 固定大小的整数数组
struct Box* ptr = malloc(..); // 手动管理内存
```

痛点：大小固定、越界不报错、自己管理内存麻烦。

C++ STL（标准模板库）提供了 **安全的动态容器**，你项目里重度使用。

---

## 1. `std::string` — 取代 `char[]`

```cpp
#include <string>

// C                          // C++
char name[100] = "hello";     std::string name = "hello";
strcat(name, " world");       name += " world";              // 直接 + 拼接
strlen(name);                 name.size();                   // 取长度，O(1)
strcmp(a, b) == 0;            a == b;                        // 直接 == 比较
// 内存手动管理                // 自动管理
```

**你项目里：**

```cpp
// config.hpp
std::string model_path = "../../models/best_rk3576_int8.rknn";
std::string camera = "/dev/video73";
std::string core_mask = "default";

// aim_writer.cpp
std::string payload = "$AIM," + std::to_string(found) + "," + state;
// 字符串拼接直接用 +，int 转 string 用 std::to_string()
```

**记忆**：`std::string` = 会自己长大的字符串，`+` 拼接、`==` 比较、`.size()` 取长、`.c_str()` 转成 C 的 `const char*`。

---

## 2. `std::vector<T>` — 取代 `T[]`（动态数组）

C 语言固定数组的替代品。**这是你项目里第二常用的类型，仅次于 cv::Mat。**

```cpp
#include <vector>

// C: 固定大小，没法扩          // C++: 动态数组
int arr[10];                     std::vector<int> nums;
                                 nums.push_back(5);    // 往里加
                                 nums.push_back(10);
                                 nums.size();          // 2
                                 nums[0];              // 5  （用 [] 和数组一样）
```

**你项目里到处都是：**

```cpp
// rknn_detector.hpp
std::vector<uint8_t> model_data_;       // 模型文件的二进制数据
std::vector<rknn_tensor_attr> output_attrs_;  // 输出张量属性列表
std::vector<OutputMatrix> matrices;     // 输出矩阵列表
std::vector<Detection> detections;      // 检测结果列表

// rknn_detector.cpp — 遍历 Detection
for (const auto& det : detections) {    // auto = 自动推导类型
    // det 是 detections 里的每个元素
}

// tracker.cpp
std::string source_ = "none";   // 跟踪来源标记
```

**记忆**：`std::vector<T>` = 能自己变大变小的数组，T 是你想存的类型。

---

## 3. `std::array<T, N>` — 取代 `T[N]`（固定大小数组）

大小在编译时就确定，比 vector 更轻量：

```cpp
#include <array>

// C: 4个浮点数的数组             // C++
float corners[4];                  std::array<float, 4> corners;

// C: 4个点的数组                  // C++
struct Point { float x, y; };
Point pts[4];                      std::array<Point, 4> pts;
```

**你项目里（rect_refiner.hpp）：**

```cpp
// rect_refiner.hpp — 返回四边形四个角点
std::array<cv::Point2f, 4> OrderCorners(const std::vector<cv::Point>& polygon);
//            ↑ 元素类型   ↑ 固定大小4
```

比 C 数组好的地方：`std::array` 知道自己的大小（`.size()`），可以传给 STL 算法。

---

## 4. `std::optional<T>` — 一个"可能有也可能没有"的盒子

**这是你项目里最有辨识度的类型。** 概念非常简单：一个盒子，里面要么装着一个值，要么是空的。

```cpp
#include <optional>

std::optional<int> find_even(int x) {
    if (x % 2 == 0) return x;        // 有结果，返回
    return std::nullopt;             // 没结果，返回"空"
}

auto result = find_even(4);
if (result.has_value()) {            // 盒子里有东西吗？
    std::cout << *result;            // 有，取出来用
}
```

**为什么不用指针？** `int*` 也能表示"有可能为空"（NULL），但：
- 指针不知道谁负责 free
- `->` 和 `*` 解引用语法不够直观
- `std::optional` 语义明确："我就是可能没有答案"

**你项目里大量使用 std::optional 来表达"检测到了吗？"：**

```cpp
// rknn_detector.hpp — 推理结果
std::optional<Detection> Infer(...);       // 可能检测到了，也可能没检测到

// tracker.hpp — 先验框
std::optional<Box> PriorBox() const;       // 有先验信息就给，没有就返回空

// rknn_detector.cpp — 选择最佳检测
std::optional<Detection> ChooseDetection(
    const std::vector<Detection>& detections,
    const std::optional<Box>& prior,       // 可能有先验框，也可能没有
    int frame_w, int frame_h) const;

// rect_refiner.hpp — 精修结果
std::optional<RefinedRect> Refine(...);    // 可能修出来了，可能失败了
```

**解析一段实际代码（rknn_detector.cpp:653-680）：**

```cpp
std::optional<Detection> RknnDetector::ChooseDetection(
    const std::vector<Detection>& detections,   // 所有检测结果
    const std::optional<Box>& prior,            // 先验框（可能为空）
    int frame_w, int frame_h) const {

    if (detections.empty()) {            // 一个检测结果都没有
        return std::nullopt;             // 返回空盒子
    }
    if (!prior.has_value()) {            // 没有先验框
        return detections.front();       // 直接返回置信度最高的那个
    }
    // 有先验框 → 从中挑一个和先验框最接近的
    // ...
    return best;  // best 也是 std::optional<Detection>
}
```

---

## 5. `std::unique_ptr<T>` — 独占所有权的指针

```cpp
#include <memory>

// C: 手动管理                      // C++: 自动管理
MyClass* p = malloc(sizeof(...));    auto p = std::make_unique<MyClass>();
// ...                               // ...
free(p);  // 容易忘!                 // 出了作用域自动释放，不会忘
```

**你项目里：**

```cpp
// main.cpp:89
auto detector = std::make_unique<RknnDetector>(config);
// detector 是独占指针：我是唯一持有者，我销毁时自动调析构函数

// main.cpp:99 — 把所有权转移给 DetectorWorker
DetectorWorker worker(std::move(detector), config.detector_affinity);
// std::move(detector) = 把 detector 的所有权交给 worker
// 此后 detector 变成 nullptr，不能再用了
```

为什么要 `std::move`？因为 `unique_ptr` 是"唯一的"，不能复制，只能转移所有权。

---

## 6. `std::mutex` / `std::atomic` — 多线程安全

线程之间共享数据，必须加锁：

```cpp
#include <mutex>
#include <atomic>

class CameraSource {
    mutable std::mutex mutex_;   // 互斥锁：同一时刻只有一个线程能拿到
    cv::Mat latest_;             // 共享帧缓存
    std::atomic<bool> stop_{false};  // 原子变量：读/写本身就是线程安全的
    std::thread thread_;         // 线程对象
};
```

**你项目里的用法：**

```cpp
// camera_source.cpp — 写线程（CaptureLoop）
{
    std::lock_guard<std::mutex> lock(mutex_);  // 拿锁
    latest_ = std::move(frame);                // 安全写入
}   // lock 出作用域，自动释放锁

// camera_source.cpp — 读线程（主循环）
bool CameraSource::ReadLatest(...) const {
    std::lock_guard<std::mutex> lock(mutex_);  // 拿锁
    *frame = latest_;                          // 安全读取
    return true;
}
```

**`std::lock_guard`** = 自动锁：构造时 lock，析构时 unlock，不会忘了解锁。

**`std::atomic<bool>`** = 读和写操作本身是原子的，两个线程同时读/写也不会出错，不需要加锁。适合简单的标志位。

---

## 7. `std::thread` — 创建线程

 C 里你创线程要用 `pthread_create`。C++ 封装了一层：

```cpp
#include <thread>

void my_function() {
    std::cout << "running in thread\n";
}

std::thread t(my_function);  // 创建一个线程，开始执行 my_function
t.join();                    // 等待线程结束
```

**你项目的线程分布在三个地方：**

```cpp
// 1. CameraSource — 相机采集线程
thread_ = std::thread(&CameraSource::CaptureLoop, this);

// 2. DetectorWorker — 推理线程
thread_ = std::thread(&DetectorWorker::Run, this);

// 3. main 线程 — 主控制循环
// （不额外创建，直接跑在 main 函数里）
```

---

## 8. `auto` — 让编译器猜类型

```cpp
// 不写 auto                         // 写 auto
std::vector<int> nums = {1,2,3};     auto nums = std::vector<int>{1,2,3};
std::unique_ptr<RknnDetector> d      auto detector
    = std::make_unique<...>(config);   = std::make_unique<RknnDetector>(config);

// 遍历 vector 的神器
for (const auto& det : detections) {   // auto 自动推导为 Detection
    // ...
}
```

**你项目里用 auto 的场景：**

```cpp
auto detector = std::make_unique<RknnDetector>(config);  // 自动推导类型
const auto result = worker.LatestResult();               // auto = InferResult
const auto detections = DecodeOutputs(matrices, meta);   // auto = vector<Detection>
auto snapshot = tracker.Snapshot();                      // auto = TrackSnapshot
```

---

## 9. `namespace` — 给代码分房间

```cpp
// C: 加前缀防止重名                  // C++：命名空间
void rkfast_box_width(...)            namespace rkfast {
void rkfast_track_update(...)             float box_width(...)
void rkfast_aim_send(...)                 void track_update(...)
                                          void aim_send(...)
                                      }
                                      // 使用: rkfast::box_width(...)
```

**你项目所有代码都在 `namespace rkfast {}` 里面：**

```cpp
namespace rkfast {

struct Box { ... };
class CameraSource { ... };

}  // namespace rkfast
```

使用了 `using namespace rkfast;` 后可以省略前缀。你 `main.cpp:65` 就写了：

```cpp
int main(int argc, char** argv) {
  using namespace rkfast;   // 后面所有 rkfast:: 前缀都可以省略
```

---

## 速查表

| C 写法 | C++ 写法 | 在项目中哪里出现 |
|--------|----------|----------------|
| `char s[100]` | `std::string s` | config.hpp, aim_writer.cpp |
| `int arr[10]`（固定） | `std::array<int, 10> arr` | rect_refiner.hpp 四角点 |
| `int* arr` + malloc | `std::vector<int> arr` | rknn_detector.cpp 模型数据 |
| `return NULL` 表示没值 | `return std::nullopt` | rknn_detector.cpp 没检测到时 |
| `malloc/free` | `std::make_unique<T>` | main.cpp detector 创建 |
| `pthread_mutex_lock` | `std::lock_guard lock(m_)` | camera_source.cpp |
| `pthread_create` | `std::thread t(func)` | camera_source.cpp |
| 函数前缀防重名 | `namespace xxx {}` | 整个项目 |
| 写死类型名 | `auto x = ...` | main.cpp 各处 |

---

## 小结

你不用背这些——看到不认识的就回这章查。最关键记住这 5 个：

1. **`std::string`** → C 的 `char[]` 替代
2. **`std::vector<T>`** → C 的动态数组
3. **`std::optional<T>`** → 可能有值也可能没有的盒子（项目灵魂类型）
4. **`std::unique_ptr<T>`** → 自动释放的指针
5. **`std::mutex` + `std::lock_guard`** → 多线程安全读写

下一章开始逐文件精读项目源码，从 `types.hpp` 开始。
