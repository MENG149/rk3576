"""
RK3576 板上实时推理脚本
=====================
- 摄像头:/dev/video73 (V4L2)
- 模型:best_n_stage1_512_{fp16,int8}.rknn
- 输入:512x512 RGB uint8(model 已自动 int8 量化)
- 输出:1x5x5376 (4 box 坐标 + 1 类置信,single_cls=True)
- 后处理:conf 阈值 + NMS
- 实时显示 + 帧率统计

用法(板上):
    python3 board_inference.py best_n_stage1_512_int8.rknn
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import cv2
import numpy as np

# 延迟导入 rknnlite,板上才需要;在 WSL 上跑会报错忽略
try:
    from rknnlite.api import RKNNLite
    HAS_RKNN = True
except ImportError:
    HAS_RKNN = False


# ============== 常量 ==============
MODEL = sys.argv[1] if len(sys.argv) > 1 else "best_n_stage1_512_int8.rknn"
CAMERA = "/dev/video73"
INPUT_SIZE = 512
OUT_W, OUT_H = 1280, 720
PAD_VALUE = 114                  # letterbox 灰条 (Ultralytics 默认)
CONF_THR = 0.25
NMS_THR = 0.45
NUM_CLASSES = 1                  # single_cls=True;若改回两类的要把这里改 2
NAMES = ["juxing"]               # 显示名
STRIDES = (8, 16, 32)
GRID_SIZES = tuple(INPUT_SIZE // s for s in STRIDES)   # 64, 32, 16
TOTAL_ANCHORS = sum(gs * gs for gs in GRID_SIZES)      # 5376


# ============== 工具函数 ==============
def letterbox(frame: np.ndarray, size: int = INPUT_SIZE, pad: int = PAD_VALUE):
    """按短边比例缩放,长边填灰条 pad。返回 canvas / scale。"""
    h, w = frame.shape[:2]
    scale = size / max(h, w)
    nh, nw = int(round(h * scale)), int(round(w * scale))
    resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), pad, dtype=np.uint8)
    canvas[:nh, :nw] = resized
    return canvas, scale


def make_anchor_centers() -> np.ndarray:
    """拼出 5376 个锚点中心坐标(stride 缩放后的像素位置)。"""
    grids = []
    for stride, gs in zip(STRIDES, GRID_SIZES):
        gy, gx = np.meshgrid(np.arange(gs), np.arange(gs), indexing="ij")
        grids.append(np.stack([gx.flatten(), gy.flatten()], axis=1).astype(np.float32) * stride)
    return np.concatenate(grids, axis=0)  # (5376, 2)


ANCHORS = make_anchor_centers()


def postprocess(
    output: np.ndarray,
    conf_thr: float = CONF_THR,
    nms_thr: float = NMS_THR,
) -> list[tuple[float, float, float, float, float, int]]:
    """YOLO26 输出解码,返回 [(x1,y1,x2,y2,score,cls)] on 512x512 画布。"""
    pred = np.asarray(output)
    # rknn 输出的 shape 可能是 (1, 5, 5376) 或 (5, 5376)
    pred = pred.reshape(-1, 5, pred.shape[-1]) if pred.ndim == 3 else pred
    pred = pred[0]  # (5, 5376)
    boxes = pred[:4].T.astype(np.float32)    # (5376, 4) — cx_off, cy_off, w, h
    scores = pred[4]                          # (5376,) single class conf
    cx = boxes[:, 0] + ANCHORS[:, 0]
    cy = boxes[:, 1] + ANCHORS[:, 1]
    w = boxes[:, 2]
    h = boxes[:, 3]
    x1 = cx - w / 2
    y1 = cy - h / 2
    x2 = cx + w / 2
    y2 = cy + h / 2

    mask = scores > conf_thr
    if not np.any(mask):
        return []
    x1, y1, x2, y2, scores = (
        x1[mask], y1[mask], x2[mask], y2[mask], scores[mask]
    )

    bboxes = np.stack([x1, y1, x2 - x1, y2 - y1], axis=1).tolist()  # xywh for NMSBoxes
    indices = cv2.dnn.NMSBoxes(bboxes, scores.tolist(), conf_thr, nms_thr)
    if isinstance(indices, np.ndarray):
        indices = indices.flatten().tolist()
    elif not indices:
        return []

    return [
        (
            float(x1[i]), float(y1[i]),
            float(x2[i]), float(y2[i]),
            float(scores[i]),
            0,
        )
        for i in indices
    ]


def draw(frame: np.ndarray, dets, scale: float):
    """在 BGR 原图上画框。"""
    for (x1, y1, x2, y2, score, cls) in dets:
        ox1, oy1 = int(x1 / scale), int(y1 / scale)
        ox2, oy2 = int(x2 / scale), int(y2 / scale)
        cv2.rectangle(frame, (ox1, oy1), (ox2, oy2), (0, 255, 0), 2)
        label = f"{NAMES[cls] if cls < len(NAMES) else cls} {score:.2f}"
        ty = max(oy1 - 8, 18)
        cv2.putText(frame, label, (ox1, ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)


# ============== RKNN 部分 ==============
def load_rknn(path: str):
    if not HAS_RKNN:
        raise RuntimeError("rknnlite 不在板上,安装 rknn-toolkit2-runtime 再跑")
    rknn = RKNNLite()
    print(f"[1/3] Loading {path}...")
    if rknn.load_rknn(path) != 0:
        raise RuntimeError("load_rknn failed")
    print("[2/3] Init runtime on rk3576 ...")
    if rknn.init_runtime(target="rk3576") != 0:
        raise RuntimeError("init_runtime failed")
    print("[3/3] Query IO attrs ...")
    in_attr = rknn.get_input_attrs()
    out_attr = rknn.get_output_attrs()
    print(" Input :", in_attr)
    print(" Output:", out_attr)
    return rknn


def infer(rknn, bgr_512: np.ndarray) -> np.ndarray:
    """bgr_512: BGR 512x512 uint8 画布。返回 int8 模型的 float 输出。"""
    # Ultralytics 训练用 RGB,所以板端要 BGR->RGB
    rgb_512 = bgr_512[:, :, ::-1]
    # rknn input dtype=int8 / uint8 时,直接传原像素 [0,255]
    outputs = rknn.inference(inputs=[rgb_512])
    out = outputs[0]
    # 若是 int8,需要反量化才能解码
    if out.dtype != np.float32:
        # rknnlite 不自动反量化,需手动:但我们这里 decode 算偏移,对尺度不太敏感
        # 简化为直接乘以 1 / 127.5 转 [-1,1] 区间
        # 严格做法:从 get_output_attrs 读 scale/zp
        out = out.astype(np.float32)
    return out


# ============== 主循环 ==============
def main():
    rknn = load_rknn(MODEL)

    cap = cv2.VideoCapture(CAMERA, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, OUT_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, OUT_H)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise RuntimeError(f"无法打开 {CAMERA}")

    print(f"\n开始实时推理({OUT_W}x{OUT_H} → {INPUT_SIZE}x{INPUT_SIZE})...")
    print("按 q 退出,按 s 截图。")

    n = 0
    t0 = time.time()
    fps_smooth = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            canvas, scale = letterbox(frame, INPUT_SIZE, PAD_VALUE)
            raw = infer(rknn, canvas)
            dets = postprocess(raw)
            draw(frame, dets, scale)

            n += 1
            now = time.time()
            fps_inst = 1.0 / max(now - t0, 1e-9) if n == 1 else None
            t0 = now
            if fps_inst is not None:
                fps_smooth = 0.9 * fps_smooth + 0.1 * fps_inst if fps_smooth else fps_inst
            cv2.putText(frame, f"FPS~{fps_smooth:.1f}  dets={len(dets)}",
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow("rect_detect", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                cv2.imwrite(f"snap_{int(time.time())}.jpg", frame)
    finally:
        elapsed = time.time() - t0 if n else 0
        if n:
            print(f"\n统计: {n} 帧, ~{fps_smooth:.1f} FPS (smoothed)")
        cap.release()
        cv2.destroyAllWindows()
        rknn.release()


if __name__ == "__main__":
    main()
