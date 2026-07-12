# RK3576 RKNN USB Targeting

Slim high-FPS RK3576 runtime for USB camera targeting.

Core path:

```text
USB camera -> RKNNLite inference -> YOLO decode/NMS -> tracker -> $AIM serial output
```

The RKNN model inference path is kept. Browser preview, JSON/binary protocols, geometry refinement, tests, and cache files are removed from the runtime version.

## Files

- `run_usb_rknn.py`: runtime entry and main loop.
- `rknn_target/rknn_runtime.py`: RKNNLite model load, NPU core mask, inference, release.
- `rknn_target/worker.py`: background RKNN inference workers.
- `rknn_target/postprocess.py`: letterbox preprocessing, YOLO decode, NMS.
- `rknn_target/io.py`: USB camera latest-frame capture and `$AIM` serial/stdout output.
- `rknn_target/tracker.py`: target smoothing and short miss prediction.
- `rknn_target/debug.py`: optional local OpenCV window.
- `tools/rk3576_performance.sh`: board performance frequency setup.
- `tools/benchmark_rk3576_modes.sh`: worker/core-mask comparison.

## Model

Default model path:

```text
models/best_rk3576_int8.rknn
```

Recommended model route:

```text
best.pt -> best.onnx -> best_rk3576_int8.rknn
static 640, batch=1, INT8, target=rk3576
```

## Board Dependencies

On RK3576 Linux:

```bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy python3-serial
pip3 install -r requirements-rk3576.txt
```

If `rknn-toolkit-lite2` cannot be installed by pip, use the matching Rockchip aarch64 wheel for the board image and Python version.

## Fast Run

Apply performance settings first:

```bash
sudo bash tools/rk3576_performance.sh
```

Run the default high-FPS command:

```bash
bash run_fast_rk3576.sh
```

Equivalent command:

```bash
python3 run_usb_rknn.py \
  --model models/best_rk3576_int8.rknn \
  --camera /dev/video73 --width 640 --height 480 --fps 60 \
  --fourcc MJPG --fast --workers 2 --worker-core-masks 0,1 \
  --no-window --output-on-result \
  --serial-port /dev/ttyUSB0 --baudrate 115200 --serial-interval 0.02
```

## Local Window

Use `--window` when you need to see the camera image and target overlay on the board:

```bash
python3 run_usb_rknn.py \
  --model models/best_rk3576_int8.rknn \
  --camera /dev/video73 --width 640 --height 480 --fps 60 \
  --fourcc MJPG --fast --workers 2 --worker-core-masks 0,1 \
  --window --display-interval 2 --output-on-result \
  --serial-port /dev/ttyUSB0 --baudrate 115200 --serial-interval 0.02
```

The default is `--no-window` because drawing slows the loop.

## Output

The only runtime protocol is one ASCII `$AIM` line:

```text
$AIM,1,LOCKED,320.4,241.6,-12.0,8.0,0.876,rknn
```

Fields:

```text
prefix,found,state,cx,cy,dx_px,dy_px,confidence,source
```

`dx_px` and `dy_px` are pixel offsets from the camera image center to the target center.

## Benchmark

Compare RKNN worker/core-mask modes:

```bash
bash tools/benchmark_rk3576_modes.sh
```

Stats include:

```text
stats frame_fps=... infer_fps=... pre=...ms rknn=...ms post=...ms select=...ms total=...ms
```

If `rknn` dominates, the RKNN model/runtime is the bottleneck. If `post` or `select` dominates, use `--fast` and `--max-det 1`.

## Verification

Local syntax check:

```bash
python -m py_compile run_usb_rknn.py rknn_target/*.py
```

Full RKNN verification must run on the RK3576 board.
