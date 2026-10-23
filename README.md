# RK3576 C++ RKNN Targeting

这是 RK3576 二维云台目标识别项目的 C++ 运行版本。仓库只保留已验证的 FP16 模型和 C++ 推理链路，不再包含旧 Python 运行代码。

当前板端验证结果：

- 识别一致性：207/207 张图片与旧输入路径一致，mismatches=0。
- RKNN 输入：默认 auto，优先使用原生 FP16 zero-copy，失败时自动回退 legacy。
- 延迟中位数：set 约 1.1 ms，total 约 13.6 ms，infer 约 73.3 FPS。
- 相机实际帧率约 27.5 FPS，当前上限来自摄像头采集。
- 性能模式使用 performance governor，不写 CPU scaling_setspeed。

## 目录

~~~text
rk3576_rknn_targeting/
|-- README.md
|-- cpp_fast_targeting/     C++ 源码、CMake、测试和运行脚本
|-- models/                 已验证的 FP16 RKNN 模型
|-- tools/                  RK3576 性能与诊断工具
+-- docs/                   指南、报告、参考资料和历史文档
~~~

唯一运行模型：

~~~text
models/best_n_stage1_512_fp16.rknn
~~~

## 板端依赖

需要 RK3576 aarch64 Linux、CMake 3.16+、支持 C++17 的编译器、OpenCV、librknnrt，以及可选的 librga。

~~~bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev
~~~

librknnrt 和 librga 应使用板卡系统或 Rockchip SDK 对应版本。

## Release 编译

在板端执行：

~~~bash
cd ~/rk3576_rknn_targeting/cpp_fast_targeting
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DRKFAST_USE_RGA=ON
cmake --build build --parallel "$(nproc)"
~~~

也可以运行完整构建检查：

~~~bash
cd ~/rk3576_rknn_targeting/cpp_fast_targeting
bash build_and_verify.sh
~~~

## 运行

默认使用 /dev/video73、NPU 双核 01、FP16 auto 输入模式，并自动应用性能 governor：

~~~bash
cd ~/rk3576_rknn_targeting/cpp_fast_targeting
bash run_camera.sh
~~~

发送串口 AIM 数据：

~~~bash
bash run_camera.sh --serial /dev/ttyS4
~~~

常用诊断参数：

~~~bash
bash run_camera.sh --no-perf
bash run_camera.sh --camera /dev/video0
bash run_camera.sh --input-mode legacy
bash run_camera.sh --core-mask 01
~~~

run_camera.sh 内的识别阈值、线程亲和性和相机参数是当前板端验证配置，不建议在未重新对比识别结果和连续多次帧率测试前修改。

## 输出

程序向标准输出或串口发送 ASCII AIM 行：

~~~text
$AIM,1,LOCKED,320.4,241.6,-12.0,8.0,0.876,rknn
~~~

字段依次为 found、state、目标中心、相对画面中心偏移、置信度和来源。

## 单元测试

测试只覆盖不依赖 NPU 的坐标、矩形精修和跟踪逻辑：

~~~bash
cmake -S cpp_fast_targeting/tests -B cpp_fast_targeting/tests/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp_fast_targeting/tests/build --parallel
ctest --test-dir cpp_fast_targeting/tests/build --output-on-failure
~~~

完整 RKNN、RGA、相机和串口验证必须在 RK3576 板端进行。

## 文档

当前指南和历史资料统一收录在 docs/，入口见 docs/README.md。性能优化细节见 docs/reports/RK3576_矩形识别修复与性能优化报告_2026-07-12.md。
我觉得练习git是
