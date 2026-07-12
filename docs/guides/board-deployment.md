# RK3576 板端部署指南

## 1. 检查模型和依赖

~~~bash
cd ~/rk3576_rknn_targeting
test -f models/best_n_stage1_512_fp16.rknn
ldconfig -p | grep rknnrt
pkg-config --modversion opencv4
~~~

需要 RGA 时继续检查：

~~~bash
ldconfig -p | grep librga
ls /usr/include/rga
~~~

## 2. 编译

~~~bash
cd ~/rk3576_rknn_targeting/cpp_fast_targeting
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DRKFAST_USE_RGA=ON
cmake --build build --parallel "$(nproc)"
~~~

若系统没有 RGA 开发库，可暂时使用 -DRKFAST_USE_RGA=OFF；识别算法不变，但预处理性能可能下降。

## 3. 运行

~~~bash
bash run_camera.sh
~~~

脚本默认执行以下有效配置：

- 模型：../models/best_n_stage1_512_fp16.rknn
- 相机：/dev/video73
- NPU：双核 01
- RKNN 输入：auto
- CPU/NPU/DDR：performance governor
- 线程亲和性：camera=5、detector=6、main=7

串口输出示例：

~~~bash
bash run_camera.sh --serial /dev/ttyS4
~~~

## 4. 常见问题

模型无法加载时，确认使用的是仓库内 FP16 模型，不要改回已验证失败的 INT8 文件。

相机无法打开时：

~~~bash
v4l2-ctl --list-devices
ls -l /dev/video*
bash run_camera.sh --camera /dev/video0
~~~

zero-copy 不受当前 RKNN runtime 支持时，auto 会自动回退。可显式比较：

~~~bash
bash run_camera.sh --input-mode auto
bash run_camera.sh --input-mode legacy
~~~

性能 governor 不允许写入时，需要 sudo；仅做普通功能诊断可使用：

~~~bash
bash run_camera.sh --no-perf
~~~
