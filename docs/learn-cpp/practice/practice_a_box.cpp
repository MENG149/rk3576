/******************************************************************************
 * 练习 A: Box 计算器进阶版
 *
 * 这个练习对应真实项目中的文件:
 *   cpp_fast_targeting/include/rkfast/types.hpp
 *
 * 真实项目里的 Box 结构体和相关函数 (BoxWidth, BoxIou 等) 都定义在 types.hpp
 * 里。本练习将这些自由函数改造为成员函数, 并增加了 IOU 和合并功能。
 *
 * 本练习覆盖的知识点:
 *   - struct (带成员函数的结构体)
 *   - 构造函数 (constructor)
 *   - 成员函数 (member function)
 *   - const 成员函数 (承诺不修改成员变量)
 *   - 函数重载 (多个同名函数, 参数不同)
 *   - std::vector 动态数组
 *   - 范围 for 循环 (range-based for loop)
 *
 * 编译: g++ -std=c++17 -o practice_a_box practice_a_box.cpp
 * 运行: ./practice_a_box
 ******************************************************************************/

#include <algorithm>  // std::max, std::min
#include <cmath>      // std::round (虽然本例未直接使用, 但常用)
#include <iomanip>    // std::setw, std::fixed, std::setprecision (格式化输出)
#include <iostream>   // std::cout, std::endl
#include <string>     // std::string, std::to_string
#include <vector>     // std::vector (动态数组, 可以随时添加/删除元素)

/******************************************************************************
 * Box 结构体 —— 对应真实项目 types.hpp 里的 struct Box
 *
 * C++ 里 struct 和 class 几乎一样, 唯一区别是:
 *   - struct 的成员默认是 public  (谁都能访问)
 *   - class  的成员默认是 private (只有自己能用)
 *
 * 在视觉项目中, Box 表示一个矩形边界框 (bounding box), 用来标出目标位置。
 * (x1, y1) 是左上角坐标, (x2, y2) 是右下角坐标。
 ******************************************************************************/
struct Box {
    // ===== 数据成员 (Data Members) =====
    // 这四个变量是 Box 的 "属性", 存储框的位置信息
    float x1;  // 左上角 x 坐标 (left)
    float y1;  // 左上角 y 坐标 (top)
    float x2;  // 右下角 x 坐标 (right)
    float y2;  // 右下角 y 坐标 (bottom)

    // ===== 构造函数 (Constructor) =====
    // 构造函数在创建对象时自动调用, 用来初始化成员变量
    // 函数名和结构体同名, 没有返回值类型
    //
    // 注意: 这里同时提供了 "默认构造函数" (无参数) 和 "带参构造函数"
    // C++ 允许函数重载 (function overloading):
    //   同一个函数名, 只要参数不同, 就可以有多个版本
    //   编译器根据调用时的参数自动选择正确的版本

    // 默认构造函数 —— 没有参数时调用
    // Box b;   ← 调用这个
    Box() : x1(0.0f), y1(0.0f), x2(0.0f), y2(0.0f) {
        // 初始化列表 (initializer list) : x1(0.0f), ... 是 C++ 推荐的初始化方式
        // 它在进入函数体之前就把成员设好, 比在 {} 里赋值更高效
        //
        // 这里是空的, 因为初始化列表已经完成了所有工作
        //
        // 通俗类比: 你点菜时直接说 "不放香菜", 比厨师做好后再把香菜挑出来更快
    }

    // 带参构造函数 —— 传入四个坐标值
    // Box b(10, 20, 100, 200);  ← 调用这个
    Box(float x1_val, float y1_val, float x2_val, float y2_val)
        : x1(x1_val), y1(y1_val), x2(x2_val), y2(y2_val) {
        // 同样是空的 —— 初始化列表已经搞定了
    }

    // ===== 成员函数 (Member Functions / Methods) =====
    //
    // 成员函数 "住" 在结构体里面, 可以直接访问成员变量, 不需要传参。
    //
    // const 修饰符: 放在参数列表后面的 const 表示 "这个函数不会修改任何成员变量"
    // 这样做的好处:
    //   1. 告诉读代码的人 "这个函数是只读的, 放心调用"
    //   2. 编译器会检查, 如果函数里不小心写了 x1 = 5, 编译直接报错
    //   3. const 对象只能调用 const 成员函数

    // 宽度 = x2 - x1
    // 对应真实项目: float BoxWidth(const Box& box) { ... }
    float width() const {
        return x2 - x1;
    }

    // 高度 = y2 - y1
    // 对应真实项目: float BoxHeight(const Box& box) { ... }
    float height() const {
        return y2 - y1;
    }

    // 面积 = 宽 x 高
    // 对应真实项目: float BoxArea(const Box& box) { ... }
    float area() const {
        return width() * height();
        // 注意: 在成员函数里调用另一个成员函数, 直接用名字即可
    }

    // 中心点 x 坐标 = (x1 + x2) / 2
    // 对应真实项目: cv::Point2f BoxCenter(const Box& box) { ... }
    float center_x() const {
        return (x1 + x2) * 0.5f;
        // 乘以 0.5 比除以 2 略快 (浮点乘法比除法快), 这是图像处理常用的微优化
    }

    // 中心点 y 坐标 = (y1 + y2) / 2
    float center_y() const {
        return (y1 + y2) * 0.5f;
    }

    // 打印框的信息到控制台
    // 这个函数不是 const, 但实际上它也不修改成员变量
    // 简单起见我们不加 const (因为 std::cout 操作本身有副作用——输出到屏幕)
    void print(const std::string& label = "") const {
        if (!label.empty()) {
            std::cout << "[" << label << "] ";
        }
        std::cout << "Box(x1=" << x1 << ", y1=" << y1
                  << ", x2=" << x2 << ", y2=" << y2 << ")"
                  << "  | 宽=" << width() << " 高=" << height()
                  << " 面积=" << area()
                  << " 中心=(" << center_x() << ", " << center_y() << ")"
                  << std::endl;
    }
};

/******************************************************************************
 * IOU (Intersection over Union) —— 交并比
 *
 * IOU 是目标检测中最重要的指标之一, 用来衡量两个边界框的 "重叠程度"。
 *
 * IOU = 交集面积 / 并集面积
 *
 *   - IOU = 0.0  → 完全不重叠
 *   - IOU = 1.0  → 完全重合
 *   - IOU = 0.5  → 一半重叠 (实际应用中常用 0.5 作为 "匹配成功" 的阈值)
 *
 * 在真实项目中, 这个函数叫 BoxIou(), 定义在 types.hpp 里。
 * 这里把它定义为外部函数 (不是 Box 的成员), 因为它操作 "两个" Box,
 * 不属于 "某一个" Box。
 *
 * 参数使用 const Box& (只读引用):
 *   - const: 保证函数内不会修改传入的 Box
 *   - & (引用): 不复制 Box, 直接访问原数据, 避免拷贝开销
 *   - 如果没有 &, 每次调用都会复制一份 Box, 对于大型对象会很慢
 ******************************************************************************/
float IOU(const Box& a, const Box& b) {
    // 第1步: 计算交集矩形 (Intersection)
    // 交集矩形的左上角 = 取两个框左上角中 "较大的 x, 较大的 y"
    // 交集矩形的右下角 = 取两个框右下角中 "较小的 x, 较小的 y"
    float ix1 = std::max(a.x1, b.x1);  // 交集的左边界 = 两个左边界中靠右的那个
    float iy1 = std::max(a.y1, b.y1);  // 交集的上边界 = 两个上边界中靠下的那个
    float ix2 = std::min(a.x2, b.x2);  // 交集的右边界 = 两个右边界中靠左的那个
    float iy2 = std::min(a.y2, b.y2);  // 交集的下边界 = 两个下边界中靠上的那个

    // 第2步: 计算交集的宽和高
    // 如果宽 <= 0 或高 <= 0, 说明两个框完全没有交集
    float inter_w = std::max(0.0f, ix2 - ix1);
    float inter_h = std::max(0.0f, iy2 - iy1);
    float inter_area = inter_w * inter_h;

    // 如果没有交集, 直接返回 0
    if (inter_area <= 0.0f) {
        return 0.0f;
    }

    // 第3步: 计算并集面积
    // 并集 = A的面积 + B的面积 - 交集面积 (因为交集被算了两次)
    float union_area = a.area() + b.area() - inter_area;

    // 第4步: 返回比值
    // 用 std::max(union_area, 1e-6f) 防止除以零
    return inter_area / std::max(union_area, 1e-6f);
    // 1e-6f = 0.000001, 一个非常小的数, 避免 division by zero
}

/******************************************************************************
 * merge_nearby —— 合并邻近的框
 *
 * 如果两个框的 IOU > 阈值 (这里取 0.3), 就把它们合并成一个大框。
 * 合并规则很简单: 取两个框的最小外接矩形。
 *
 * 在真实项目中虽然没有这个精确的函数, 但它模拟了检测后处理中的
 * NMS (Non-Maximum Suppression, 非极大值抑制) 的简化版思想:
 *   重叠度高的检测框 → 合并成一个。
 *
 * 参数说明:
 *   const Box& a, const Box& b  — 只读引用
 *   float threshold = 0.3f      — 默认参数 (default argument)
 *     如果调用时不给 threshold, 自动用 0.3
 *     可以这样调用: merge_nearby(a, b)       → threshold = 0.3
 *     也可以这样:   merge_nearby(a, b, 0.5f) → threshold = 0.5
 ******************************************************************************/
Box merge_nearby(const Box& a, const Box& b, float threshold = 0.3f) {
    // 先计算 IOU, 判断是否应该合并
    float iou_value = IOU(a, b);

    std::cout << "  IOU(" << a.center_x() << "," << a.center_y()
              << " vs " << b.center_x() << "," << b.center_y()
              << ") = " << iou_value;

    if (iou_value > threshold) {
        // IOU 超过阈值 → 合并成最小外接矩形
        // 合并规则:
        //   新的 x1 取两个 x1 中较小的 (最左边)
        //   新的 y1 取两个 y1 中较小的 (最上边)
        //   新的 x2 取两个 x2 中较大的 (最右边)
        //   新的 y2 取两个 y2 中较大的 (最下边)
        Box merged(
            std::min(a.x1, b.x1),  // 最左
            std::min(a.y1, b.y1),  // 最上
            std::max(a.x2, b.x2),  // 最右
            std::max(a.y2, b.y2)   // 最下
        );
        std::cout << " -> 合并! (阈值=" << threshold << ")" << std::endl;
        return merged;
    } else {
        // IOU 不足 → 不合并, 返回一个 "无效框" (面积为0)
        std::cout << " -> 不合并 (阈值=" << threshold << ")" << std::endl;
        return Box(0, 0, 0, 0);
        // 调用者可以通过 area() == 0 来判断合并是否成功
    }
}

/******************************************************************************
 * main 函数 —— 程序入口
 *
 * 测试上面定义的所有功能:
 *   1. 创建 3 个 Box 对象
 *   2. 测试成员函数 (width, height, area, center_x, center_y)
 *   3. 测试 IOU 计算
 *   4. 测试合并功能
 *   5. 用 std::vector<Box> 存储多个框并遍历打印
 ******************************************************************************/
int main() {
    // 设置浮点数输出格式: 保留 1 位小数
    std::cout << std::fixed << std::setprecision(1);

    // ================================================================
    // 第1部分: 创建 Box 对象并测试成员函数
    // ================================================================

    std::cout << "========== 第1部分: 创建 Box 并测试成员函数 =========="
              << std::endl;

    // 使用带参构造函数创建 3 个 Box
    // 这模拟了目标检测中检测到的 3 个候选框
    Box box1(10.0f, 20.0f, 110.0f, 220.0f);  // 宽100, 高200
    Box box2(50.0f, 60.0f, 150.0f, 260.0f);  // 宽100, 高200, 与box1部分重叠
    Box box3(300.0f, 400.0f, 350.0f, 450.0f); // 窄50, 高50, 远离前两个

    // 调用成员函数并打印
    // 每个 Box 对象用 . 操作符访问成员函数
    box1.print("box1  ");
    box2.print("box2  ");
    box3.print("box3  ");

    std::cout << std::endl;

    // 单独测试每个成员函数
    std::cout << "--- 单独测试成员函数 ---" << std::endl;
    std::cout << "box1.width()    = " << box1.width() << std::endl;
    std::cout << "box1.height()   = " << box1.height() << std::endl;
    std::cout << "box1.area()     = " << box1.area() << std::endl;
    std::cout << "box1.center_x() = " << box1.center_x() << std::endl;
    std::cout << "box1.center_y() = " << box1.center_y() << std::endl;

    std::cout << std::endl;

    // ================================================================
    // 第2部分: 测试 IOU 计算
    // ================================================================

    std::cout << "========== 第2部分: 测试 IOU 计算 ==========" << std::endl;

    // box1 和 box2 有重叠 → IOU 应该在 0.3 ~ 0.7 之间
    float iou_12 = IOU(box1, box2);
    std::cout << "IOU(box1, box2) = " << iou_12 << std::endl;

    // box1 和 box3 不重叠 → IOU 应该为 0
    float iou_13 = IOU(box1, box3);
    std::cout << "IOU(box1, box3) = " << iou_13 << " (期望 0.0)" << std::endl;

    std::cout << std::endl;

    // ================================================================
    // 第3部分: 测试合并功能
    // ================================================================

    std::cout << "========== 第3部分: 测试合并 (merge_nearby) =========="
              << std::endl;

    // box1 和 box2 的 IOU 应该 > 0.3 → 会被合并
    std::cout << "尝试合并 box1 和 box2:" << std::endl;
    Box merged_12 = merge_nearby(box1, box2);
    if (merged_12.area() > 0.0f) {
        merged_12.print("合并结果");
    } else {
        std::cout << "  未合并 (IOU 不足)" << std::endl;
    }

    std::cout << std::endl;

    // box1 和 box3 的 IOU = 0 → 不会被合并
    std::cout << "尝试合并 box1 和 box3:" << std::endl;
    Box merged_13 = merge_nearby(box1, box3);
    if (merged_13.area() > 0.0f) {
        merged_13.print("合并结果");
    } else {
        std::cout << "  未合并 (IOU 不足)" << std::endl;
    }

    std::cout << std::endl;

    // 测试自定义阈值:
    // 如果把阈值设得很低 (比如 0.05), 即使几乎不重叠也会合并
    std::cout << "尝试用极低阈值 (0.05) 合并 box1 和 box3:" << std::endl;
    Box merged_low = merge_nearby(box1, box3, 0.05f);
    if (merged_low.area() > 0.0f) {
        merged_low.print("低阈值合并");
    }

    std::cout << std::endl;

    // ================================================================
    // 第4部分: 使用 std::vector<Box> 批量管理框
    // ================================================================
    //
    // std::vector<T> 是 C++ 的动态数组, 对应 C 语言的 malloc + realloc
    // 但它是自动管理内存的 (RAII — Resource Acquisition Is Initialization)
    // 你可以随意添加/删除元素, 它会自动调整大小
    //
    // 在真实项目中:
    //   std::vector<Detection> detections = ...;  // 存储所有检测结果
    //   detections[i].box 就是这里的 Box

    std::cout << "========== 第4部分: std::vector<Box> 批量操作 =========="
              << std::endl;

    // 创建一个 vector, 把之前创建的 box 都放进去
    // push_back() 是 vector 最常用的方法, 在末尾添加一个元素
    std::vector<Box> boxes;
    boxes.push_back(box1);
    boxes.push_back(box2);
    boxes.push_back(box3);

    // 范围 for 循环 (range-based for loop) —— C++11 引入的语法糖
    // 语法: for (元素类型 变量名 : 容器名) { ... }
    // 编译器自动遍历容器中的每一个元素, 不需要写 i < boxes.size()
    //
    // const auto& 的含义:
    //   const  — 只读, 不修改元素
    //   auto   — 让编译器自动推断类型 (这里 auto = Box)
    //   &      — 引用, 避免拷贝 (如果不加 &, 每次循环都会复制一份 Box)
    //
    // 通俗理解: "对于 boxes 里面的每一个 box, 执行下面的代码"

    std::cout << "遍历 vector 中的所有 Box:" << std::endl;
    for (const auto& box : boxes) {
        // box 是当前遍历到的元素 (只读引用)
        std::cout << "  ";
        box.print("检测框");
    }

    // 再演示一些 vector 的其他常用操作
    std::cout << "\nvector 的其他操作:" << std::endl;
    std::cout << "  boxes.size()     = " << boxes.size() << std::endl;
    std::cout << "  boxes.empty()    = " << (boxes.empty() ? "true" : "false")
              << std::endl;

    // 用下标访问 (和 C 数组一样)
    std::cout << "  boxes[0].area()  = " << boxes[0].area() << std::endl;
    std::cout << "  boxes[1].width()  = " << boxes[1].width() << std::endl;

    // front() 返回第一个元素, back() 返回最后一个元素 (都是引用)
    std::cout << "  boxes.front().center_x() = " << boxes.front().center_x()
              << std::endl;
    std::cout << "  boxes.back().center_x()  = " << boxes.back().center_x()
              << std::endl;

    // 清空 vector
    // boxes.clear();
    // std::cout << "  清空后 size = " << boxes.size() << std::endl;

    std::cout << std::endl;

    // ================================================================
    // 第5部分: 综合演示 —— 模拟检测后处理
    // ================================================================
    //
    // 在真实项目中, 检测器会输出多个候选框, 然后需要:
    //   1. 计算两两之间的 IOU
    //   2. 把重叠度高的合并 (或做 NMS)
    //   3. 选出最佳目标
    //
    // 下面用刚学的知识做一个简化版:

    std::cout << "========== 第5部分: 综合演示 — 模拟检测后处理 =========="
              << std::endl;

    // 模拟检测器输出的 5 个候选框
    std::vector<Box> detections;
    detections.push_back(Box(100, 100, 200, 200));  // 框 A
    detections.push_back(Box(120, 110, 190, 210));  // 框 B: 与 A 高度重叠
    detections.push_back(Box(130, 120, 180, 190));  // 框 C: 与 A, B 高度重叠
    detections.push_back(Box(400, 100, 500, 200));  // 框 D: 远处的另一个目标
    detections.push_back(Box(410, 105, 490, 195));  // 框 E: 与 D 高度重叠

    std::cout << "原始检测框 (" << detections.size() << " 个):" << std::endl;
    for (size_t i = 0; i < detections.size(); ++i) {
        // size_t 是无符号整数类型, 专门用来表示大小和索引
        // 用 char('A' + i) 把索引转成字母 A, B, C, D, E
        std::cout << "  框 " << char('A' + static_cast<int>(i)) << ": ";
        detections[i].print();
    }

    std::cout << "\n逐对计算 IOU:" << std::endl;
    for (size_t i = 0; i < detections.size(); ++i) {      // 外层: 第 i 个框
        for (size_t j = i + 1; j < detections.size(); ++j) { // 内层: 第 j 个框 (j > i, 避免重复)
            float iou = IOU(detections[i], detections[j]);
            std::cout << "  IOU(" << char('A' + static_cast<int>(i))
                      << ", " << char('A' + static_cast<int>(j))
                      << ") = " << iou << std::endl;
        }
    }

    std::cout << "\n结论: A-B-C 聚为一组 (高 IOU), D-E 聚为另一组 (高 IOU),"
              << std::endl;
    std::cout << "      两组之间 IOU=0 (不同目标)。" << std::endl;

    std::cout << "\n========== 练习 A 完成 ==========" << std::endl;
    std::cout << "你学会了: struct, 构造函数, 成员函数, IOU, vector, 范围for"
              << std::endl;
    std::cout << "下一步: 练习 B — 模拟目标跟踪器 (类 + α-β滤波 + 状态机)"
              << std::endl;

    return 0;
}
