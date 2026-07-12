# RK3576 RKNN 输入零拷贝优化设计

日期：2026-07-12
状态：用户已批准

## 目标与边界

优化 cpp_fast_targeting 的 RKNN 输入提交路径，消除每帧
rknn_inputs_set() 约 10 ms 的固定开销，同时保证模型识别效果不变。

本次只优化输入 I/O。以下内容保持不变：

- 模型 models/best_n_stage1_512_fp16.rknn。
- RGA/OpenCV 图像预处理及输入 RGB 字节。
- rknn_outputs_get()、FP16 解码和后处理。
- 置信度阈值、选框、OpenCV 精修和跟踪参数。

## 当前基线

板端为 RK3576、aarch64、Linux 6.1.99，librknnrt 版本 2.3.2。
使用 /dev/video73、NPU core mask 01 和 Release 构建时，稳定基线为：

    frame_fps=27.5 infer_fps=43.5~44.5
    pre=2.1ms set=10.0ms rknn=10.0~10.1ms get=0.1ms
    post=0.2ms total=22.4~22.5ms

get 已经只有约 0.1 ms。本次优化目标是 set。

## 输入零拷贝设计

1. RknnDetector::Open() 查询 RKNN_QUERY_NATIVE_INPUT_ATTR，并确认本模型的
   原生输入为 NHWC FLOAT16。
2. 使用 rknn_create_mem() 创建 1.5 MB 可复用输入内存，以 pass_through=1
   调用 rknn_set_io_mem() 绑定一次。
3. 每帧继续执行原有预处理，得到与旧路径完全相同的 UINT8 RGB 数据。
4. 使用 256 项查找表将 UINT8 除以 255 并精确转换为 FP16，直接写入绑定内存，
   再调用 rknn_mem_sync(..., RKNN_MEMORY_SYNC_TO_DEVICE)。
5. 不再逐帧调用 rknn_inputs_set()；rknn_run()、rknn_outputs_get() 和后处理
   代码保持不变。
6. Close() 在销毁 RKNN context 前调用 rknn_destroy_mem()。

## 运行模式与回退

新增参数：

    --rknn-input-mode auto|legacy|zero-copy

- legacy：强制使用现有 rknn_inputs_set() 路径，用于基线和回退。
- zero-copy：强制使用零拷贝；初始化或同步失败时直接报错，禁止静默回退。
- auto：初始化时尝试零拷贝；创建或绑定失败则清理资源、打印原因并回退 legacy。

零拷贝初始化成功后，运行期间不在同一个 context 上临时切换 I/O 模式。
全部精度和性能验收通过后，auto 成为默认值；不支持该原生张量路径的运行时
会在初始化阶段自动回退 legacy。

零拷贝模式的 set 字段记录“输入 memcpy + cache sync”的完整耗时。不能只看 set：
如果驱动把工作移动到 rknn_run()，必须以端到端 total 判断是否真正优化。

## 最高性能策略

tools/rk3576_performance.sh 修改为：

- 删除 CPU userspace governor 和 scaling_setspeed 固定频率写入。
- CPU policy0 和 policy4 仅使用 performance governor。
- 保留 CPU 深度 idle 禁用，以降低唤醒抖动。
- NPU devfreq 使用 performance governor，板端最高频为 950 MHz。
- DDR devfreq 使用 performance governor。
- 结束时打印 CPU、NPU、DDR 当前 governor 和频率；设置失败必须明确报告。

run_camera.sh 默认通过 sudo 应用该性能策略；新增 --no-perf，供无 sudo 的诊断
环境显式跳过。性能脚本或 sudo 执行失败时，启动脚本必须返回非零状态，不能
继续运行并假装已启用最高性能模式。

NPU 默认使用 core mask 01，明确同时启用 RK3576 的 core 0 和 core 1。板端实测：

    01:  total=22.4~22.5ms，稳定
    all: total=22.7ms，偶发升至 24.2ms

RKNN 将 all 定义为运行时自动选择，并不比明确双核更强，因此不作为默认值。

## 识别效果验收

使用本机 dataset123/dataset/images/val 的全部 207 张图片，在同一块开发板、
同一个 FP16 模型和相同参数下分别运行 legacy 与 zero-copy。

逐图通过条件：

- 是否检出完全一致。
- 类别 ID 完全一致。
- 最终置信度绝对误差不超过 1e-4。
- 最终框每个坐标绝对误差不超过 0.1 像素。

新增 input_mode_compare 工具，在一个进程中建立 legacy 和 zero-copy 两个
RknnDetector，按相同顺序逐张推理并输出机器可读结果。出现差异时打印图片名、
旧结果和新结果。任一图片失败即判定精度验收失败，不启用零拷贝默认路径。
两条路径还需记录送入 RKNN 前的有效输入字节数和校验值，排除预处理数据差异。

## 性能验收

两种路径使用相同 governor、NPU 核心、模型、相机配置和无窗口模式。warmup 后
各运行至少三轮，每轮不少于 10 秒，以中位数比较。

通过条件：

- zero-copy 的 set 中位数小于 2 ms。
- zero-copy 的 total 中位数相对基线至少降低 5 ms。
- rknn 增长没有抵消 set 收益。
- 无 RKNN、cache sync、内存释放或相机错误。

相机当前限制约 27.5 FPS，因此主要比较 infer_fps 和 total，不以 frame_fps
作为本次主要成功指标。

## 板端验收结果

全部 207 张验证图片通过：输入校验值、是否检出、类别、置信度和框坐标均一致，
mismatches=0。

相同 governor、模型、相机和 NPU 双核设置下，各运行三轮 12 秒后的中位数：

    legacy:    infer=43.6 FPS, set=10.0ms, rknn=10.1ms, total=22.8ms
    zero-copy: infer=73.3 FPS, set=1.1ms,  rknn=9.8ms,  total=13.6ms

零拷贝端到端减少 9.2 ms，推理吞吐提升约 68%。相机帧率仍受硬件限制为
27.5 FPS。

## 构建与验证顺序

1. 在板端使用 Release 和现有 -O3 构建。
2. 编译 rkfast_target、image_probe、input_mode_compare 和现有单元测试。
3. 运行 rkfast_tests。
4. 对 207 张图片生成 legacy 和 zero-copy 结果并严格比较。
5. 运行至少三轮相机性能 A/B 测试。
6. 检查初始化回退、强制零拷贝失败和正常关闭的资源释放。

## 预计改动范围

- cpp_fast_targeting/include/rkfast/config.hpp
- cpp_fast_targeting/src/config.cpp
- cpp_fast_targeting/include/rkfast/rknn_detector.hpp
- cpp_fast_targeting/src/rknn_detector.cpp
- cpp_fast_targeting/tools/input_mode_compare.cpp
- cpp_fast_targeting/CMakeLists.txt
- cpp_fast_targeting/run_camera.sh
- tools/rk3576_performance.sh
- 相关运行文档

不修改 RKNN 模型、训练数据、阈值、后处理算法或跟踪参数。
