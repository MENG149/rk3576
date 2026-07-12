# 05 -- CPU 亲和性与线程绑定

## RK3576 的大小核架构

RK3576 这颗芯片有 8 个 CPU 核心，但它们不是平等的：

| 核心编号 | 类型 | 特点 | 适合的活 |
|----------|------|------|----------|
| 0-3 | Cortex-A53（小核） | 主频低、功耗低、性能弱 | I/O 等待、后台任务 |
| 4-7 | Cortex-A72（大核） | 主频高、功耗高、性能强 | 计算密集型任务 |

这就是 ARM 的 **big.LITTLE 架构**（大小核）。手机芯片也是这样设计的：刷微信用大核，待机用小核。

**为什么不所有核都用大核？** 一是贵，二是功耗和发热扛不住。嵌入式设备（你的比赛设备）尤其要考虑功耗，所以大小混搭是最务实的方案。

在这个项目里，CPU 绑定是**性能优化的重要手段**。如果不绑定，Linux 调度器可能把你的推理线程扔到小核上跑，结果一帧推理从 20ms 变成 80ms，全乱套了。

---

## 头文件：thread_affinity.hpp

```cpp
#pragma once

#include <string>

namespace rkfast {

bool BindCurrentThread(const std::string& cpu_spec, std::string* error = nullptr);

}  // namespace rkfast
```

这个头文件极其简洁，只暴露一个函数：`BindCurrentThread`。

参数解释：
- **cpu_spec**：类型是 `std::string`，CPU 规格字符串。比如 `"4-7"` 表示"绑定到 4、5、6、7 号核心"，`"0,2"` 表示"绑定到 0 号和 2 号核心"。
- **error**：`std::string*`（C++ 中的字符串指针），用于输出错误信息。默认值是 `nullptr`，如果你不关心错误，可以完全不传这个参数。如果绑定失败，函数会通过这个指针把原因写回去，方便调试。

返回值是 `bool`：绑定成功返回 `true`，失败返回 `false`。

---

## ParseCpuSpec：解析 CPU 规格字符串

这是文件里的匿名 namespace 函数（只在 `thread_affinity.cpp` 内部可见，对外隐藏）：

```cpp
namespace {

std::vector<int> ParseCpuSpec(const std::string& spec) {
  std::vector<int> cpus;
  std::stringstream ss(spec);
  std::string token;
  while (std::getline(ss, token, ',')) {
```

**第一段：初始化**

- `std::vector<int> cpus` -- 准备一个空的整数数组，用来装解析出来的 CPU 编号。
- `std::stringstream ss(spec)` -- `stringstream` 是 `<sstream>` 头文件里的类，作用是把一个字符串当成"流"来读。你可以把它想象成一个在内存中的文件，可以按逗号分割、按行读取。这里把 `spec` 字符串塞进流里。
- `std::string token` -- 用来接收每次分割出来的片段。
- `std::getline(ss, token, ',')` -- 从流里读取，遇到 `,` 就停。每次循环 token 拿到逗号之间的内容。比如 `"4-7,0,2"` 三次循环分别拿到 `"4-7"`、`"0"`、`"2"`。

**第二段：去空格**

```cpp
    token.erase(std::remove_if(token.begin(), token.end(),
                               [](unsigned char ch) { return std::isspace(ch) != 0; }),
                token.end());
    if (token.empty()) {
      continue;
    }
```

这段看起来吓人，但其实只干了一件事：**把 token 里的空白字符（空格、Tab）全部删掉**。

逐层拆解：
- `std::remove_if(begin, end, 条件)` -- 一个 STL 算法，把容器中"满足条件"的元素全移到末尾，返回指向"新末尾"的迭代器。**注意 `remove_if` 不真的删除，只是把需要保留的元素挪到前面。**
- `[](unsigned char ch) { return std::isspace(ch) != 0; }` -- 这是一个 **lambda 表达式**（C++11 引入的匿名函数）。`[]` 是 lambda 的捕获列表（这里为空，不捕获外部变量），`(unsigned char ch)` 是参数，`{...}` 是函数体。`std::isspace(ch)` 判断字符是否是空白。整句翻译："如果这个字符是空白，把它移走"。
- `token.erase(...)` -- 真正删除从"新末尾"到 `token.end()` 之间的元素（就是那些被移过去的空白字符）。

最终效果：`" 4-7 "` 变成 `"4-7"`；空格全部消失。

`if (token.empty()) continue;` -- 如果去空格后什么都没剩下（比如遇到 `",,"` 两个逗号之间是空的），跳过。

**第三段：处理单个数字或范围**

```cpp
    const auto dash = token.find('-');
    if (dash == std::string::npos) {
      cpus.push_back(std::stoi(token));
      continue;
    }
```

- `token.find('-')` -- 在 token 里找 `-` 的位置。如果是 `"3"`（纯数字），`dash` 等于 `std::string::npos`（一个特殊常量，表示"没找到"）。
- 如果没找到 `-`，说明这是一个单独的 CPU 编号。`std::stoi(token)` 把字符串转成 int（比如 `"3"` → `3`），然后 `push_back` 加到结果数组里。

**第四段：处理 `4-7` 范围语法**

```cpp
    const int begin = std::stoi(token.substr(0, dash));
    const int end = std::stoi(token.substr(dash + 1));
    if (begin <= end) {
      for (int cpu = begin; cpu <= end; ++cpu) {
        cpus.push_back(cpu);
      }
    } else {
      for (int cpu = begin; cpu >= end; --cpu) {
        cpus.push_back(cpu);
      }
    }
```

- `token.substr(0, dash)` -- 取 `-` 前面的部分。比如 `"4-7"` → `"4"`。
- `token.substr(dash + 1)` -- 取 `-` 后面的部分。`"4-7"` → `"7"`。
- `std::stoi(...)` 转成 int。

然后判断 `begin <= end`：
- **正常情况**（`"4-7"` → begin=4, end=7）：for 循环从 4 递增到 7，依次 push_back（4, 5, 6, 7）。
- **倒序情况**（`"7-4"` → begin=7, end=4）：从 7 递减到 4，也支持。这种写法实际很少用，但代码兼容了。

最终返回 `cpus`，里面装着所有解析出来的核心编号。

---

## BindCurrentThread：把当前线程锁到指定核心

```cpp
bool BindCurrentThread(const std::string& cpu_spec, std::string* error) {
  if (cpu_spec.empty()) {
    return true;
  }
```

如果 `cpu_spec` 是空字符串（比如配置里写 `core_mask = ""`），什么都不做直接返回成功。这表示"不绑定，让系统自由调度"。

**第二步：解析规格字符串**

```cpp
  const std::vector<int> cpus = ParseCpuSpec(cpu_spec);
  if (cpus.empty()) {
    if (error != nullptr) {
      *error = "empty cpu affinity spec: " + cpu_spec;
    }
    return false;
  }
```

调用上面刚学的 ParseCpuSpec，拿到核心编号列表。如果解析结果为空（比如传了 `","`），返回错误。

**第三步：操作 cpu_set_t -- Linux 的 CPU 掩码**

```cpp
  cpu_set_t set;
  CPU_ZERO(&set);
  for (const int cpu : cpus) {
    if (cpu >= 0) {
      CPU_SET(cpu, &set);
    }
  }
```

这里出现了三个新东西，逐个解释：

**`cpu_set_t`** -- 这是一个 C 语言定义的结构体（来自 `<sched.h>`），本质上是一个**位图（bitmap）**。8 个核心就是 8 个 bit，每个 bit 代表一个核心是否被选中：

```
        bit 7 6 5 4 3 2 1 0
set =     [0 0 0 0 1 1 1 1]   ← 表示选中核心 0,1,2,3
```

**`CPU_ZERO(&set)`** -- 宏，把 set 的所有 bit 全部清零。`&set` 是取地址，因为这是 C 的宏，需要传指针进去让它修改。

**`CPU_SET(cpu, &set)`** -- 宏，把 set 的第 `cpu` 个 bit 置为 1。循环执行完后，`set` 里只有我们指定的核心被标记为 1。

`if (cpu >= 0)` 是一个安全检查，防止负值把位图搞乱。

**第四步：真正绑定**

```cpp
#ifdef __linux__
  const int ret = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
  if (ret != 0 && error != nullptr) {
    *error = "pthread_setaffinity_np failed: " + std::string(std::strerror(ret));
  }
  return ret == 0;
#else
  (void)set;
  if (error != nullptr) {
    *error = "thread affinity is only supported on Linux";
  }
  return false;
#endif
}
```

`#ifdef __linux__` 是**条件编译**：在 Linux 上编译时走上面的代码，在 Windows/Mac 上编译时走 `#else` 分支（直接返回 false，因为 pthread 亲和性 API 是 Linux 特有的）。

Linux 分支里：

- **`pthread_self()`** -- 获取**当前线程**的 ID（pthread_t 类型）。注意：谁调用 `BindCurrentThread`，就绑谁。
- **`pthread_setaffinity_np(d, n, s)`** -- Linux 系统调用，把线程绑定到指定 CPU 核心集。三个参数：
  - `thread_id`：pthread_self() 返回的当前线程 ID
  - `cpusetsize`：cpu_set_t 的大小（`sizeof(set)`）
  - `cpuset`：指向 cpu_set_t 的指针
  - `_np` 后缀：**non-portable**（不可移植），表示这是 Linux 特有的扩展，不是 POSIX 标准的一部分。
- **`std::strerror(ret)`** -- 把错误码（int）转成人类可读的错误信息（比如 "No such process"）。
- `(void)set;` -- 在非 Linux 分支里，set 变量会被警告"未使用"，这行代码告诉编译器"我是故意不用的，别警告"。

---

## 三个线程分别绑到哪

打开项目的 main.cpp（或配置文件），能看到三个线程的绑定策略：

```cpp
// config.hpp 里的默认配置
std::string core_mask = "default";           // 配置文件里：main_core_mask
std::string detector_affinity = "4-7";       // 推理线程 → 大核
std::string camera_affinity = "0-3";         // 采集线程 → 小核
```

三条线程的数据流是这样的：

```
  ┌─────────────────┐      ┌─────────────────┐      ┌─────────────────┐
  │ CameraSource    │      │  DetectorWorker │      │  Main Thread    │
  │ (采集线程)       │ ────→│  (推理线程)      │ ────→│  (主控制循环)    │
  │ CPU: 0-3 (小核) │      │ CPU: 4-7 (大核) │      │ CPU: 4-7 (大核) │
  │                 │      │                 │      │                 │
  │ → ReadLatest()  │      │ → Submit()      │      │ → 跟踪          │
  │                 │      │ → Infer()       │      │ → ROI精修       │
  │                 │      │ → DecodeOutputs │      │ → 串口输出       │
  └─────────────────┘      └─────────────────┘      └─────────────────┘
```

主线程也绑到大核，因为跟踪、ROI 精修代码逻辑虽不重但跑在主循环里，绑到大核保证响应速度。

---

## 为什么 camera 绑小核，detector 绑大核

**CameraSource 绑小核（0-3）的原因：**

相机采集是典型的 **I/O 密集型任务**。工作流程大致是：

1. 调用 V4L2 驱动去读 USB/CSI 摄像头的一帧数据 -- 大部分时间在**等待硬件完成数据传输**。
2. 数据到达后，把 YUYV 转成 BGR（颜色空间转换，OpenCV 内部做），这个计算量不大（相对推理而言）。
3. 更新帧缓存，通知主线程有新帧可用。

关键点：采集线程的 CPU 大部分时间其实在**等待**（等摄像头送数据过来，等 DMA 传输完成）。给它分配大核纯属浪费 -- 你让一个能跑马拉松的运动员去排队买咖啡，太奢侈了。小核主频低、功耗低，对付这种"等一会儿干一会儿"的活绰绰有余。

**DetectorWorker 绑大核（4-7）的原因：**

RKNN 推理是典型的 **计算密集型任务**：

1. 预处理：RGA（2D 硬件加速器）做 resize 和颜色转换 -- 虽然 RGA 是独立硬件，但驱动调用、DMA 配置都在 CPU 上跑。
2. 推理：`.rknn` 模型载入 NPU（神经网络处理器）执行前向推理 -- NPU 做矩阵运算，但 CPU 负责喂数据、读取结果、控制流程。
3. 后处理：解码输出张量，跑 NMS 算法 -- 纯 CPU 计算，包括大量 IOU 运算、排序、筛选。

大核的指令发射宽度更大、Cache 更大、SIMD 单元更宽，同样的 NMS 代码在大核上可能快 2-3 倍。在 RK3576 上实测，detector 绑大核能稳定跑到 80+ FPS，绑小核可能掉到 30 FPS。

**一句话总结**：I/O 等硬件的活给小核，自己算的活给大核。这就是大小核架构下 CPU 绑定的根本原则。

---

## 速查表

| 概念 | 说明 | 在哪里 |
|------|------|--------|
| `cpu_set_t` | Linux CPU 位图，每个 bit 代表一个核心 | `<sched.h>` |
| `CPU_ZERO(&set)` | 位图全部清零 | `<sched.h>` 宏 |
| `CPU_SET(n, &set)` | 位图第 n 位置 1 | `<sched.h>` 宏 |
| `pthread_setaffinity_np` | 把线程绑到指定 CPU 集 | `<pthread.h>`，Linux 特有 |
| `pthread_self()` | 获取当前线程 ID | `<pthread.h>` |
| `std::stringstream` | 把字符串当流来读（按字符分割） | `<sstream>` |
| `std::getline(ss, token, ',')` | 从流里按逗号分割读取 | `<string>` |
| `std::remove_if` | 把满足条件的元素移到末尾 | `<algorithm>` |
| lambda 表达式 `[](){ }` | 匿名函数，就地定义 | C++11 语法 |
| `std::stoi(s)` | 字符串转 int | `<string>` |
| `#ifdef __linux__` | 条件编译：只在 Linux 上编译 | 预处理器指令 |
| `std::strerror(err)` | 错误码转人类可读字符串 | `<cstring>` |

---

## 小结

CPU 亲和性绑定在嵌入式高性能场景下不是"锦上添花"而是"必需品"。这篇文档覆盖了三个层次：

1. **硬件层**：RK3576 有 8 核，0-3 小核省电，4-7 大核高性能。把你的任务放到对的核上，性能才能拉满。
2. **系统调用层**：`cpu_set_t` 位图 + `pthread_setaffinity_np` 完成实际绑定，这是 Linux 提供的底层能力。
3. **C++ 封装层**：`ParseCpuSpec` 把 `"4-7"` 这种人类可读的字符串转成 cpu_set_t，`BindCurrentThread` 把当前线程绑上去。都是薄薄一层封装，核心逻辑不到 50 行。

你在比赛时如果发现推理 FPS 不稳定，第一个要排查的就是：`cat /proc/<pid>/status | grep Cpus_allowed_list` 看看推理线程绑到大核了没有。

---

## 练习 C：手写 ParseCpuSpec

新建 `practice_c_affinity.cpp`：

```cpp
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cctype>

// ===== 你要实现的核心函数 =====
std::vector<int> ParseCpuSpec(const std::string& spec) {
    std::vector<int> cpus;
    std::stringstream ss(spec);
    std::string token;

    while (std::getline(ss, token, ',')) {
        // 去空格
        token.erase(std::remove_if(token.begin(), token.end(),
                                   [](unsigned char ch) { return std::isspace(ch); }),
                    token.end());
        if (token.empty()) continue;

        auto dash = token.find('-');
        if (dash == std::string::npos) {
            // 单个数字
            cpus.push_back(std::stoi(token));
        } else {
            // 范围 "4-7"
            int begin = std::stoi(token.substr(0, dash));
            int end   = std::stoi(token.substr(dash + 1));
            if (begin <= end) {
                for (int cpu = begin; cpu <= end; ++cpu) cpus.push_back(cpu);
            } else {
                for (int cpu = begin; cpu >= end; --cpu) cpus.push_back(cpu);
            }
        }
    }
    return cpus;
}

int main() {
    // 测试1：范围语法
    auto result1 = ParseCpuSpec("4-7");
    std::cout << "\"4-7\" → ";
    for (int cpu : result1) std::cout << cpu << " ";    // 预期 4 5 6 7
    std::cout << "\n";

    // 测试2：混合语法
    auto result2 = ParseCpuSpec("0, 2, 4-7");
    std::cout << "\"0, 2, 4-7\" → ";
    for (int cpu : result2) std::cout << cpu << " ";    // 预期 0 2 4 5 6 7
    std::cout << "\n";

    // 测试3：带空格
    auto result3 = ParseCpuSpec(" 0-3 , 7 ");
    std::cout << "\" 0-3 , 7 \" → ";
    for (int cpu : result3) std::cout << cpu << " ";    // 预期 0 1 2 3 7
    std::cout << "\n";

    return 0;
}
```

编译运行：

```bash
g++ -std=c++17 -o practice_c practice_c_affinity.cpp && ./practice_c
```

预期输出：

```
"4-7" → 4 5 6 7
"0, 2, 4-7" → 0 2 4 5 6 7
" 0-3 , 7 " → 0 1 2 3 7
```

在 Linux 板上运行时，可以加上真实绑定测试（需要链接 pthread）：

```cpp
// 在 main 里加上这几行（仅 Linux 有效）：
#ifdef __linux__
#include <pthread.h>
#include <sched.h>
// 把前面 main 的最后一段改成：
    auto result4 = ParseCpuSpec("4-7");
    cpu_set_t set;
    CPU_ZERO(&set);
    for (int cpu : result4) CPU_SET(cpu, &set);
    int ret = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    std::cout << "Bind result: " << (ret == 0 ? "OK" : "FAIL") << "\n";
#endif
```

编译时加 `-lpthread`：

```bash
g++ -std=c++17 -o practice_c_linux practice_c_affinity.cpp -lpthread && ./practice_c_linux
```
