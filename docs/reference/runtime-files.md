# 当前运行文件

- cpp_fast_targeting/CMakeLists.txt：C++ Release 构建入口。
- cpp_fast_targeting/include/rkfast/：公共类型和模块接口。
- cpp_fast_targeting/src/：相机、RKNN、后处理、跟踪和输出实现。
- cpp_fast_targeting/run_camera.sh：板端运行入口。
- cpp_fast_targeting/build_and_verify.sh：板端构建和依赖检查。
- cpp_fast_targeting/tests/：不依赖 NPU 的 C++ 单元测试。
- cpp_fast_targeting/tools/：静态图片探针和输入模式对比工具。
- models/best_n_stage1_512_fp16.rknn：唯一已验证运行模型。
- tools/rk3576_performance.sh：performance governor 配置。
- tools/rk3576_monitor.sh：板端状态监控。
- tools/test_rga.sh：RGA 诊断。

旧 Python 运行代码已删除。docs/archive 中的命令和模型名仅用于历史追溯。
