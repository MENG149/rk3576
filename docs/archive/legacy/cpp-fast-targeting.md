# RK3576 C++ Fast Targeting

This subproject is a performance-first C++ runtime for the existing `rk3576_rknn_targeting` project.

Pipeline:

```text
USB camera -> async RKNN detection -> ROI rectangle refine -> tracker -> $AIM output
```

The point of this runtime is not full-frame YOLO on every frame. The point is to keep targeting output fast by using:

- coarse RKNN detection only when needed
- small ROI rectangle refine on most frames
- short prediction when detections are skipped or missed

## Build On RK3576

Install board-side packages first:

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev
```

Then build:

```bash
cd ~/rk3576_rknn_targeting/cpp_fast_targeting
mkdir -p build
cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DRKNN_API_INCLUDE_DIR=/usr/include \
  -DRKNN_API_LIB=/usr/lib/librknnrt.so
cmake --build . -j4
```

Adjust `RKNN_API_INCLUDE_DIR` and `RKNN_API_LIB` to match your board image if needed.

If the board already has `/usr/lib/librknnrt.so` but does not have `rknn_api.h`,
you can place the header directly in:

```text
cpp_fast_targeting/third_party/rknn/include/rknn_api.h
```

The CMake file checks that local path before system include directories.

If CMake still says it cannot find OpenCV, check:

```bash
pkg-config --modversion opencv4
pkg-config --cflags --libs opencv4
```

If those commands fail, `libopencv-dev` is still missing on the board.

## Run

Apply the existing board performance script first:

```bash
cd ~/rk3576_rknn_targeting
sudo bash tools/rk3576_performance.sh
```

Then run:

```bash
cd ~/rk3576_rknn_targeting/cpp_fast_targeting/build
./rkfast_target \
  --model ../../models/best_rk3576_int8.rknn \
  --camera /dev/video73 \
  --width 640 --height 480 --fps 60 \
  --fourcc MJPG \
  --input-size 640 \
  --core-mask default \
  --class-count 2 \
  --target-class 0 \
  --conf 0.45 \
  --max-det 1 \
  --detect-interval-locked 2 \
  --serial-port /dev/ttyUSB0 \
  --baudrate 115200
```

For CPU pinning on RK3576, try this after checking `lscpu -e` first:

```bash
./rkfast_target \
  --model ../../models/best_yolo26n_512_rk3576_int8.rknn \
  --camera /dev/video73 \
  --width 640 --height 480 --fps 60 \
  --fourcc MJPG \
  --input-size 512 \
  --core-mask default \
  --class-count 1 \
  --target-class 0 \
  --max-det 1 \
  --detect-interval-locked 2 \
  --main-affinity 4-7 \
  --detector-affinity 4-7 \
  --camera-affinity 0-3 \
  --no-window
```

That example assumes the big cores are `4-7` and the small cores are `0-3`.

For pure benchmark without the serial link:

```bash
./rkfast_target \
  --model ../../models/best_rk3576_int8.rknn \
  --camera /dev/video73 \
  --width 640 --height 480 --fps 60 \
  --fourcc MJPG \
  --max-det 1 \
  --detect-interval-locked 2
```

## Check Single-Core vs Dual-Core RKNN

Use the same command and change only `--core-mask`. Keep the camera, model,
input size, and window setting identical:

```bash
cd ~/rk3576_rknn_targeting/cpp_fast_targeting/build

for mask in 0 1 01 auto; do
  echo "===== core-mask ${mask} ====="
  timeout 20s ./rkfast_target \
    --model ../../models/best_yolo26n_512_rk3576_int8.rknn \
    --camera /dev/video73 \
    --width 640 --height 480 --fps 60 \
    --fourcc MJPG \
    --input-size 512 \
    --class-count 1 \
    --target-class 0 \
    --max-det 1 \
    --no-window \
    --core-mask "${mask}"
done
```

Read the printed stats:

```text
stats frame_fps=... infer_fps=... pre=...ms rknn=...ms post=...ms total=...ms
```

Interpretation:

- If `--core-mask 01` has much lower `rknn` time than `0` or `1`, one
  inference can use dual-core cooperative execution.
- If `01` is similar to `0` and `1`, this model/runtime is effectively
  single-core per RKNN context.
- `frame_fps` is capped by the camera FPS. Use `infer_fps` and `rknn=...ms`
  to compare NPU behavior.

## Notes

- Start with `640x480` capture, `640` model input, `max_det=1`.
- If `rknn` time is still too high, the next step is a smaller model input such as `512`.
- If display hurts throughput, keep the default `--no-window`.
- This first C++ drop keeps the project small and direct. RGA/DMA-BUF can be layered on later.
