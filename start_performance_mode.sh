#!/bin/bash
# RK3576 Performance Mode Startup Script
# 启动性能模式并运行RKNN目标跟踪程序

set -e

echo "====================================="
echo "RK3576 Performance Mode Startup"
echo "====================================="
echo ""

# Step 1: Enable Performance Mode
echo "[1/4] Setting CPU/GPU/NPU to performance mode..."

if [ -f "tools/rk3576_performance.sh" ]; then
    sudo bash tools/rk3576_performance.sh
else
    echo "Warning: tools/rk3576_performance.sh not found, setting manually..."

    # CPU Governor to performance
    echo "Setting CPU governor to performance..."
    for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
        if [ -f "$cpu" ]; then
            echo performance | sudo tee "$cpu" > /dev/null
        fi
    done

    # Set CPU to max frequency
    for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_max_freq; do
        if [ -f "$cpu" ]; then
            max_freq=$(cat "${cpu%/*}/cpuinfo_max_freq")
            echo "$max_freq" | sudo tee "$cpu" > /dev/null
        fi
    done

    # GPU to performance
    if [ -f "/sys/class/devfreq/fb000000.gpu/governor" ]; then
        echo performance | sudo tee /sys/class/devfreq/fb000000.gpu/governor > /dev/null
    fi

    # NPU to performance
    if [ -f "/sys/class/devfreq/fdab0000.npu/governor" ]; then
        echo performance | sudo tee /sys/class/devfreq/fdab0000.npu/governor > /dev/null
    fi
fi

echo "✓ Performance mode enabled"
echo ""

# Step 2: Check RGA device
echo "[2/4] Checking RGA hardware..."
if [ -e "/dev/rga" ]; then
    sudo chmod 666 /dev/rga
    echo "✓ RGA device ready: /dev/rga"
else
    echo "⚠ Warning: /dev/rga not found, RGA acceleration may not work"
fi
echo ""

# Step 3: Build with RGA if needed
echo "[3/4] Checking build status..."
if [ ! -f "cpp_fast_targeting/build/rkfast_target" ]; then
    echo "Executable not found, building with RGA..."
    cd cpp_fast_targeting
    bash build_rga.sh
    cd ..
    echo "✓ Build complete"
else
    echo "✓ Executable found: cpp_fast_targeting/build/rkfast_target"
fi
echo ""

# Step 4: Run the program
echo "[4/4] Starting RKNN targeting program..."
echo "====================================="
echo ""

# Default parameters (modify as needed)
MODEL_PATH="models/best_rk3576_int8.rknn"
CAMERA_DEV="/dev/video73"
WIDTH=640
HEIGHT=480
FPS=60
FOURCC="MJPG"
CORE_MASK="01"  # Use 2 NPU cores
STATS_INTERVAL=2.0

# Check if model exists
if [ ! -f "$MODEL_PATH" ]; then
    echo "Error: Model not found at $MODEL_PATH"
    echo "Please specify correct model path"
    exit 1
fi

# Check if camera exists
if [ ! -e "$CAMERA_DEV" ]; then
    echo "Warning: Camera device $CAMERA_DEV not found"
    echo "Available video devices:"
    ls -l /dev/video* 2>/dev/null || echo "No video devices found"
    echo ""
    read -p "Enter camera device path (e.g., /dev/video0): " CAMERA_DEV
fi

echo "Configuration:"
echo "  Model: $MODEL_PATH"
echo "  Camera: $CAMERA_DEV"
echo "  Resolution: ${WIDTH}x${HEIGHT} @ ${FPS}fps"
echo "  FOURCC: $FOURCC"
echo "  NPU Cores: $CORE_MASK"
echo ""
echo "Starting in 2 seconds... (Ctrl+C to cancel)"
sleep 2

cd cpp_fast_targeting/build

./rkfast_target \
  --model "../../$MODEL_PATH" \
  --camera "$CAMERA_DEV" \
  --width $WIDTH \
  --height $HEIGHT \
  --fps $FPS \
  --fourcc $FOURCC \
  --core-mask $CORE_MASK \
  --no-window \
  --stats-interval $STATS_INTERVAL \
  --output-on-result

echo ""
echo "Program exited."
