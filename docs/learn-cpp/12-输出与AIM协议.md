# 12 -- 输出与 AIM 协议：把跟踪结果发出去

## 这一章讲什么

前面几章我们把检测、精修、跟踪全做完了，得到的是一个 `TrackSnapshot`（当前目标的状态快照）。但这东西只在程序内存里，外面没人知道。这一章讲**怎么把结果发出去**，让下位机（比如 STM32 云台）能收到。

负责这件事的类叫 `AimWriter`，在 `aim_writer.hpp/cpp` 里。

---

## AimWriter 的"三件套"接口

打开头文件，看到的就是三个公开函数：

```cpp
class AimWriter {
public:
    explicit AimWriter(const AppConfig& config);
    ~AimWriter();

    bool Open(std::string* error);   // 开门
    void Close();                     // 关门
    void Send(const TrackSnapshot& snapshot, int frame_w, int frame_h);  // 发数据
};
```

这个结构你在前面几个类里已经见过很多次了 -- **Open / 干活 / Close** 三件套。构造时只存配置，Open 时才真正打开设备，Close 时释放资源，析构函数自动调 Close（又是 RAII，"死了自动关"）。

区别在哪？AimWriter **不一定有硬件**。如果你没配串口，Open 直接返回成功，但 `fd_` 还是 -1。后面的 Send 看到 `fd_ < 0` 就改走 stdout。

---

## 双通道输出策略：串口优先，stdout 兜底

这是 AimWriter 最巧妙的设计 -- **同一个 Send 函数，根据有没有串口走不同路径**：

```
Send(snapshot)
    │
    ├── fd_ >= 0 ?  ──YES──→ 串口发送（给下位机）
    │
    └── fd_ < 0  ?  ──YES──→ std::cout 打印（给人看）
```

对应的配置项在 `config.hpp` 里：

```cpp
std::string serial_port;       // 串口设备路径，比如 "/dev/ttyUSB0"
int baudrate = 115200;         // 波特率
double stdout_interval = 0.20;  // stdout 输出间隔（秒）
double serial_interval = 0.02;  // 串口输出间隔（秒）
```

如果 `serial_port` 为空字符串，就不打开串口，纯 stdout 输出。这在调试阶段非常有用 -- 你没有硬件串口也能看到完整的 $AIM 输出。

---

## Open 函数：Linux 系统调用实战

```cpp
bool AimWriter::Open(std::string* error) {
    if (config_.serial_port.empty()) {
        return true;   // 没配串口，直接成功
    }
    fd_ = open(config_.serial_port.c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        if (error != nullptr) {
            *error = "open serial failed: " + config_.serial_port + " : " +
                     std::strerror(errno);
        }
        return false;
    }
    if (!ConfigureSerial(fd_, error)) {
        fd_ = -1;
        return false;
    }
    return true;
}
```

### open() 系统调用

这就是 Linux 的 `open()`，和 C 语言用的一模一样。C++ 只是外面包了一层类。

```c
// C 语言你会这么写：
int fd = open("/dev/ttyUSB0", O_WRONLY | O_NOCTTY | O_NONBLOCK);
if (fd < 0) { perror("open"); exit(1); }
```

三个标志位（flag）的含义：

| 标志 | 含义 | 大白话 |
|------|------|--------|
| `O_WRONLY` | Write Only | 只写模式。我们只往外发数据，不需要读串口 |
| `O_NOCTTY` | No Controlling TTY | 不要让这个串口变成当前 shell 的控制终端。不加这个标志的话，万一串口连了什么设备发来乱码，你的程序就当终端指令执行了，很危险 |
| `O_NONBLOCK` | Non-Blocking | 非阻塞模式。`write()` 调用时如果设备缓冲区满了，**不等待，直接返回失败**。避免主循环被串口卡住 |

这三个标志用 `|`（按位或）组合在一起传给 open。如果用标准 C 写，写法完全一样 -- 这些都是 POSIX 标准，不是 C++ 特有的。

**为什么是只写（O_WRONLY）不是读写（O_RDWR）？**

因为我们这个场景只需要**往外发** AIM 协议数据，不需要从下位机接收反馈。如果将来你要做双向通信（比如发完命令等下位机回复"收到"），再改成 `O_RDWR`。

---

## ConfigureSerial：串口配置的灵魂

如果 open 只是"把门推开"，ConfigureSerial 就是"把房间内部布置好"。串口通信不像网络通信那样开箱即用 -- 波特率、数据位、校验位、停止位都得手动配。

```cpp
bool AimWriter::ConfigureSerial(int fd, std::string* error) const {
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        // 获取当前配置失败
        close(fd);
        return false;
    }

    // --- 第1步：设置波特率 ---
    cfsetospeed(&tty, ToBaud(config_.baudrate));   // 输出波特率
    cfsetispeed(&tty, ToBaud(config_.baudrate));   // 输入波特率

    // --- 第2步：配置控制标志 ---
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;      // 8 数据位
    tty.c_cflag |= (CLOCAL | CREAD);                  // 忽略调制解调器，启用接收
    tty.c_cflag &= ~(PARENB | PARODD);                // 无校验位
    tty.c_cflag &= ~CSTOPB;                           // 1 停止位
    tty.c_cflag &= ~CRTSCTS;                          // 无硬件流控

    // --- 第3步：配置本地和输出标志 ---
    tty.c_lflag = 0;     // 关闭所有本地模式（不回显、不信号处理）
    tty.c_oflag = 0;     // 关闭所有输出处理（直接透传）

    // --- 第4步：配置输入标志 ---
    tty.c_iflag &= ~IGNBRK;                                 // 不忽略 BREAK 信号
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                 // 无软件流控

    // --- 第5步：非阻塞读取配置 ---
    tty.c_cc[VMIN] = 0;     // 最少读 0 个字节就返回
    tty.c_cc[VTIME] = 0;    // 不等，立即返回

    // --- 第6步：应用配置 ---
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return false;
    }
    return true;
}
```

### termios 结构体 -- 串口配置的"总开关"

`termios` 是 Linux 提供的标准结构体，定义在 `<termios.h>` 里。它是串口所有配置的集合体。C 程序员应该很熟了，C++ 里也是完全相同的用法。

核心成员：

| 成员 | 全称 | 管什么 |
|------|------|--------|
| `c_cflag` | Control Flags | 串口硬件参数：波特率、数据位、校验位、停止位、流控 |
| `c_iflag` | Input Flags | 输入处理：是否转义回车、是否处理 XON/XOFF |
| `c_oflag` | Output Flags | 输出处理：是否把换行映射成回车换行、是否做字符替换 |
| `c_lflag` | Local Flags | 本地模式：是否回显、是否规范模式、是否产生信号 |
| `c_cc[]` | Control Characters | 特殊控制字符数组，`VMIN` 和 `VTIME` 在这里 |

六步配置拆解：

**第1步：波特率**

```cpp
cfsetospeed(&tty, ToBaud(config_.baudrate));  // 输出速度
cfsetispeed(&tty, ToBaud(config_.baudrate));  // 输入速度
```

`ToBaud()` 是一个辅助函数，把数字转成 termios 定义的常量：

```cpp
speed_t ToBaud(int baudrate) {
    switch (baudrate) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200:
        default:     return B115200;   // 不知道的就默认 115200
    }
}
```

输入输出速度可以不一样（比如用不同的上传/下载速率），但一般设成一样的。

**第2步：控制标志（c_cflag）**

这一行是串口最基本的装配：

```cpp
tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
```

- `~CSIZE` 把当前数据位设置清空
- `| CS8` 设置为 8 位数据位

组合起来：**8 个数据位（CS8）、无校验位（清 PARENB）、1 个停止位（清 CSTOPB）、无硬件流控（清 CRTSCTS）**。

这就是最常见、最标准的串口配置，俗称 "8N1"（8 位数据，No parity，1 停止位）。

```cpp
tty.c_cflag |= (CLOCAL | CREAD);
```

- `CLOCAL`：忽略调制解调器控制线（DCD 信号）。如果你用过老式电话线拨号上网就懂这个。现在串口设备（USB 转串口等）没人用调制解调器了，但你不设这个标志的话，open 可能因为 DCD 信号不正常而失败。
- `CREAD`：启用接收。虽然我们只是写数据，但底层驱动需要这个标志才能正常工作。

**第3步：本地和输出标志 -- 全部关掉**

```cpp
tty.c_lflag = 0;   // 全关
tty.c_oflag = 0;   // 全关
```

把 `c_lflag` 清 0 很重要。默认情况下串口会：
- 回显（你发的数据回显给你） --> 关掉
- 把 `\n` 转成 `\r\n` --> 关掉
- 按 Ctrl+C 发信号 --> 关掉

我们是发 AIM 协议数据的，不需要这些终端特性。全部关掉，**透传**。

同理 `c_oflag = 0`，不把换行映射成回车换行，不做任何字符转换。

**第4步：输入标志**

```cpp
tty.c_iflag &= ~IGNBRK;                              // 收到 BREAK 信号要汇报
tty.c_iflag &= ~(IXON | IXOFF | IXANY);               // 关闭软件流控
```

软件流控（XON/XOFF）是一种用 ASCII 字符（Ctrl+S / Ctrl+Q）来控制数据流的方法，但现在已经很少用了。关掉。

**第5步：非阻塞读取 -- VMIN=0, VTIME=0**

```cpp
tty.c_cc[VMIN] = 0;     // 最少读 0 个字节就返回（不等）
tty.c_cc[VTIME] = 0;    // 超时 0 秒（不超时等待）
```

这两个参数控制 `read()` 的行为。因为是只写模式（O_WRONLY），我们其实不读。但配一下是好习惯，万一将来改成读写模式，read 不会卡住。

- `VMIN=0, VTIME=0`：read 立即返回，有多少读多少，没有就返回 0。这是纯粹的非阻塞模式。

**第6步：应用配置**

```cpp
tcsetattr(fd, TCSANOW, &tty);
```

`TCSANOW` = 立即生效，不等。还有一种 `TCSADRAIN` 是等当前缓冲区发完再改，我们用不上。

---

## FormatAim：把跟踪快照变成一行字符串

这是 AIM 协议的编码端：

```cpp
std::string AimWriter::FormatAim(const TrackSnapshot& snapshot,
                                  int frame_w, int frame_h) {
    const int found = snapshot.active ? 1 : 0;
    const float cx = snapshot.active ? snapshot.center.x : 0.0f;
    const float cy = snapshot.active ? snapshot.center.y : 0.0f;
    const float dx = snapshot.active
                         ? snapshot.center.x - frame_w * 0.5f
                         : 0.0f;
    const float dy = snapshot.active
                         ? snapshot.center.y - frame_h * 0.5f
                         : 0.0f;
    char buffer[256];
    std::snprintf(buffer, sizeof(buffer),
                  "$AIM,%d,%s,%.1f,%.1f,%.1f,%.1f,%.3f,%s",
                  found,
                  snapshot.state.c_str(),
                  cx, cy,
                  dx, dy,
                  snapshot.conf,
                  snapshot.source.c_str());
    return std::string(buffer);
}
```

### $AIM 协议格式

输出一行字符串，字段用逗号分隔：

```
$AIM,found,state,cx,cy,dx,dy,conf,source
```

每个字段的含义：

| 位置 | 字段名 | 类型 | 含义 | 取值范围 |
|------|--------|------|------|----------|
| 1 | `$AIM` | -- | 协议头，固定字符串 | 永远 $AIM |
| 2 | `found` | int | 是否检测到目标 | 0=没目标, 1=有目标 |
| 3 | `state` | string | 跟踪状态 | "SEARCHING"=搜索中, "TRACKING"=跟踪中, "LOST"=丢失 |
| 4 | `cx` | float（1位小数） | 目标中心 X 坐标（像素） | 0 ~ 640（以 640 为例） |
| 5 | `cy` | float（1位小数） | 目标中心 Y 坐标（像素） | 0 ~ 480（以 480 为例） |
| 6 | `dx` | float（1位小数） | 目标中心相对画面中心的 X 偏移（像素） | -320 ~ +320 |
| 7 | `dy` | float（1位小数） | 目标中心相对画面中心的 Y 偏移（像素） | -240 ~ +240 |
| 8 | `conf` | float（3位小数） | 置信度 | 0.000 ~ 1.000 |
| 9 | `source` | string | 来源 | "none", "rknn", "roi", "roi+rknn" |

### 字段详解

**found（检测标志）**

`snapshot.active` 是 `TrackSnapshot` 里的布尔值。active=true 表示当前有活跃目标，active=false 表示目标丢失或还没找到。FormatAim 把它转成整数 1 或 0：

```cpp
const int found = snapshot.active ? 1 : 0;
```

**cx / cy（绝对坐标）**

目标中心在画面中的像素坐标。左上角是 (0,0)，右下角是 (frame_w-1, frame_h-1)。

如果没目标（active=false），坐标全填 0.0。

```cpp
const float cx = snapshot.active ? snapshot.center.x : 0.0f;
```

**dx / dy（偏移坐标）**

这是**下位机云台控制最关心的两个值**。dx 是目标中心相对于画面中心的水平偏移：

```cpp
const float dx = snapshot.active
                     ? snapshot.center.x - frame_w * 0.5f
                     : 0.0f;
```

- dx > 0：目标在画面中心右边，云台需要向右转
- dx < 0：目标在画面中心左边，云台需要向左转
- dy > 0：目标在画面中心下方，云台需要向下转
- dy < 0：目标在画面中心上方，云台需要向上转
- dx=0, dy=0：目标在正中间，云台锁定

下位机读到 dx, dy 后直接输给 PID 控制器就能追目标了，不需要再做坐标换算。

**conf（置信度）**

来自 RKNN 推理的置信度输出。值越大说明模型越确定"这里确实有个目标"。通常大于 0.45（config_.conf_threshold）才会被当成有效检测。

**state（跟踪状态）**

这是 `RectTracker` 维护的状态机：
- `SEARCHING`：搜索模式，还没找到目标
- `TRACKING`：正在稳定跟踪目标
- `LOST`：目标丢了，正在尝试找回

**source（数据来源）**

标识本次跟踪结果是从哪来的：
- `rknn`：纯检测结果（第一次检测到，或者精修失败了）
- `roi`：纯 ROI 精修结果（没新检测，只靠图像精修维持跟踪）
- `roi+rknn`：检测+精修联合（最好的情况，又有检测又修好了）
- `none`：没任何数据输入，纯预测

### 一例完整的 $AIM 输出行

```
$AIM,1,TRACKING,320.0,240.0,0.0,0.0,0.952,roi+rknn
```

解析：检测到了目标（found=1），正在跟踪中（TRACKING），目标正好在画面正中央（cx=320, cy=240, dx=0, dy=0），置信度 0.952，来源是检测+精修联合。

```
$AIM,0,SEARCHING,0.0,0.0,0.0,0.0,0.000,none
```

解析：没有目标（found=0），正在搜索（SEARCHING），所有坐标都是 0，没有置信度，没有来源。

---

## Send 函数：双重节流策略

```cpp
void AimWriter::Send(const TrackSnapshot& snapshot, int frame_w, int frame_h) {
    const std::string payload = FormatAim(snapshot, frame_w, frame_h);
    const double now = NowSeconds();

    if (fd_ >= 0) {
        // === 串口通道 ===
        if (config_.serial_interval > 0.0 &&
            now - last_serial_ < config_.serial_interval) {
            return;   // 距离上次发送还不满 0.02 秒，跳过
        }
        const std::string line = payload + "\n";
        (void)!write(fd_, line.data(), line.size());   // 发出去，忽略错误
        last_serial_ = now;
        return;
    }

    // === stdout 通道 ===
    if (now - last_stdout_ >= config_.stdout_interval) {
        std::cout << payload << std::endl;
        last_stdout_ = now;
    }
}
```

### 串口通道：高频节流（默认 0.02 秒 = 50Hz）

串口直接连下位机 STM32。我们**每帧都调 Send**，但不一定每帧都真发。节流逻辑：

```
if (now - last_serial_ < 0.02) return;   // 还不够 20ms，不发
```

默认 `serial_interval = 0.02` 秒，意味着**串口最多 50Hz**发送。你的相机可能是 60fps，每 16.7ms 一帧，每帧都调 Send，但串口最多发 50 次/秒。中间被跳过的帧不心疼 -- 下位机收到上一帧的数据还没处理完，你硬塞新数据反而可能让它缓冲区溢出。

`write()` 返回值的奇怪写法：

```cpp
(void)!write(fd_, line.data(), line.size());
```

- `write()` 返回实际写入的字节数，-1 表示出错
- `!` 把返回值转成"是否出错"（0=成功, 1=失败）
- `(void)` 表示"我知道有返回值但我故意不用"

实际效果：发了就发了，如果出错也不卡主循环。在竞赛场景下串口可能偶尔掉帧，但不能因为串口出错就影响整个程序的实时性。

### stdout 通道：低速节流（默认 0.2 秒 = 5Hz）

如果没接串口（`fd_ < 0`），就把 $AIM 打印到终端：

```cpp
if (now - last_stdout_ >= config_.stdout_interval) {
    std::cout << payload << std::endl;
}
```

默认 `stdout_interval = 0.2` 秒，**stdout 最多 5Hz**。

### 为什么 stdout 间隔更大（0.2s vs 0.02s）？

两个原因：

1. **终端打印本身有开销**。`std::cout << std::endl` 会触发 flush，把缓冲区刷到屏幕上。串口写入只是往内核缓冲区拷贝几个字节（几十纳秒），终端打印是 I/O 操作（可能几百微秒到毫秒级）。如果每帧都打印（60Hz），终端刷屏本身就会拖慢主循环。

2. **给人看的，不需要那么快**。串口给下位机，50Hz 才有意义（控制频率要够快）。stdout 是你在调试时用眼睛看的，5Hz 足够了 -- 人眼的反应速度也就是 100ms 左右，再快了你也看不清刷屏。

你可以把 `stdout_interval` 调成 1.0 秒（1Hz），调试时看着更舒服。

---

## 和 C 语言对比

如果用标准 C 写完整的串口发送模块，代码大概是这样的：

```c
// ===== C 语言版本：串口发送模块 =====

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

// 全局变量 -- C 没有类，只能全局或放结构体里
static int g_fd = -1;
static const char* g_port = NULL;

// 波特率转换（和 C++ 版本一模一样）
speed_t to_baud(int baudrate) {
    switch (baudrate) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200:
        default:     return B115200;
    }
}

// 配置串口
int configure_serial(int fd, int baudrate) {
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, to_baud(baudrate));
    cfsetispeed(&tty, to_baud(baudrate));

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag &= ~IGNBRK;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

// 打开串口
int serial_open(const char* port, int baudrate) {
    // 注意：必须分配内存存端口名，或者调用者保证指针存活
    g_port = strdup(port);   // ← 手动分配，容易忘 free

    int fd = open(port, O_WRONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "open %s: %s\n", port, strerror(errno));
        return -1;
    }

    if (configure_serial(fd, baudrate) != 0) {
        close(fd);
        return -1;
    }

    g_fd = fd;
    return 0;
}

// 关闭串口
void serial_close(void) {
    if (g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }
    free((void*)g_port);   // ← 必须手动释放
    g_port = NULL;
}

// 发送 AIM 数据
void serial_send(const char* payload) {
    if (g_fd < 0) {
        printf("%s\n", payload);   // 没串口就打印
        return;
    }

    char line[512];
    snprintf(line, sizeof(line), "%s\n", payload);
    write(g_fd, line, strlen(line));   // 不管返回值，发了就行
}
```

对比 C++ 版本，C 版本的额外负担：

| 问题 | C | C++ |
|------|-----|------|
| 内存管理 | `strdup` 要 `free`，容易忘 | `std::string` 自动管理 |
| 状态保护 | 全局变量谁都能改 | `private` 成员，外面碰不到 |
| 资源释放 | 必须手动调 `serial_close` | 析构函数自动调 `Close` |
| 组织方式 | 函数散落在外面，不知道谁管谁 | 类把函数和数据打包在一起 |
| 错误处理 | `return -1`，调用者容易忽略 | `std::string* error` 返回详细描述 |
| 节流逻辑 | 要手动维护全局 `last_send` 变量 | 封装在类内部，调用者不用管 |

其实核心的 POSIX 系统调用（`open`、`write`、`tcgetattr`、`tcsetattr`）C 和 C++ 写法完全一样。区别在于**组织方式** -- C++ 用类把状态（fd_、last_serial_、last_stdout_）和行为（Open、Send、Close）打包在一起，外部调用者只需要三行：

```cpp
AimWriter writer(config);
writer.Open(&error);
while (running) {
    writer.Send(snapshot, frame_w, frame_h);  // 节流、渠道选择全自动
}
writer.Close();  // 其实析构函数会自动调，写了更安全
```

---

## 速查表

| 概念 | 一句话 |
|------|--------|
| `O_WRONLY` | 只写模式打开设备文件 |
| `O_NOCTTY` | 不要把这个设备当成终端 |
| `O_NONBLOCK` | 写缓冲区满了也不等，直接返回 |
| `termios` | Linux 串口配置的总结构体 |
| `CS8` | 8 个数据位 |
| `PARENB` 清掉 | 无校验位 |
| `CSTOPB` 清掉 | 1 个停止位 |
| `CLOCAL` | 忽略调制解调器，现代串口必加 |
| `CREAD` | 启用接收（就算只写也需要） |
| `VMIN=0 VTIME=0` | 非阻塞读取，不等待 |
| `$AIM` | 协议头，固定字符串 |
| `dx/dy` | 目标中心相对画面中心的偏移，云台 PID 控制就靠它 |
| `serial_interval=0.02` | 串口最多 50Hz |
| `stdout_interval=0.20` | 终端最多 5Hz，给人看的 |

---

## 小结

这一章你把 AIM 协议的输出路线走通了：

1. **AimWriter** 类也只有 **Open/Close/Send** 三个公开接口，和其他类一个模子
2. **双通道**：有串口发串口，没串口发 stdout，同一个 Send 接口自动选
3. **串口配置**全在 ConfigureSerial 里，termios 结构体 -- 波特率、8N1、非阻塞、无流控 -- 标准配置一套走完
4. **$AIM 协议**就是一行逗号分隔的文本：`$AIM,found,state,cx,cy,dx,dy,conf,source`
5. **dx/dy** 是云台控制的核心：目标离画面中心有多远，下位机直接输给 PID
6. **节流**：串口 0.02s（50Hz），stdout 0.2s（5Hz）。串口快是为了控制精度，stdout 慢是因为终端打印贵且人看不过来

现在你已经看完了一个从相机采集到串口输出的完整链路。下一章（第13章）是总复习，把 main 函数从头到尾串起来讲，看看这些模块是怎么协作的。
