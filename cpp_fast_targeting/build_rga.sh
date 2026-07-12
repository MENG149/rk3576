#!/bin/bash
# Build script for RK3576 with RGA hardware acceleration

set -e

echo "=== Building RK3576 RKNN Targeting with RGA ==="

# Clean previous build
if [ -d "build" ]; then
  echo "Cleaning previous build..."
  rm -rf build
fi

mkdir -p build
cd build

echo "Configuring with CMake..."
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DRKFAST_USE_RGA=ON \
  -DCMAKE_CXX_FLAGS="-march=native -mtune=cortex-a76"

echo "Building..."
make -j$(nproc)

echo ""
echo "=== Build Complete ==="
echo "Executable: build/rkfast_target"
echo ""
echo "Run with:"
echo "  ./rkfast_target --model ../models/best_rk3576_int8.rknn \\"
echo "    --camera /dev/video73 --width 640 --height 480 --fps 60 \\"
echo "    --fourcc MJPG --no-window"
echo ""
echo "Expected improvements:"
echo "  - Preprocessing: 9.0ms → 1-2ms (RGA acceleration)"
echo "  - Frame FPS: 27.5 → 45-55 FPS"
echo ""
