#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

python -u run_usb_rknn.py \
  --model models/best_rk3576_int8.rknn \
  --camera /dev/video73 \
  --width 640 --height 480 --fps 60 \
  --fourcc MJPG \
  --fast \
  --workers 2 \
  --worker-core-masks 0,1 \
  --protocol aim \
  --output-on-result \
  --serial-port /dev/ttyUSB0 \
  --baudrate 115200 \
  --serial-interval 0.02 \
  --preview-port 8080 \
  --preview-fps 10 \
  --preview-quality 70
