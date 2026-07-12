#!/bin/bash
# RGA Hardware Test and Diagnostic Tool

set -u
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_ROOT"

echo "====================================="
echo "RK3576 RGA Hardware Diagnostic"
echo "====================================="
echo ""

# 1. Check RGA devices in sysfs
echo "[1] RGA Platform Devices:"
ls -l /sys/bus/platform/devices/ | grep rga
echo ""

# 2. Check device nodes
echo "[2] RGA Device Nodes:"
ls -l /dev/rga* 2>/dev/null || echo "  No /dev/rga* found"
ls -l /dev/dri/ 2>/dev/null || echo "  No /dev/dri/ found"
echo ""

# 3. Check RGA in /proc/devices
echo "[3] Character Devices:"
cat /proc/devices | grep -i rga || echo "  No RGA in /proc/devices"
echo ""

# 4. Check RGA driver info
echo "[4] RGA Driver Info:"
for rga_dev in /sys/devices/platform/27920000.rga /sys/devices/platform/27930000.rga; do
    if [ -d "$rga_dev" ]; then
        echo "  Device: $rga_dev"
        echo "    Driver: $(readlink $rga_dev/driver 2>/dev/null || echo 'none')"
        echo "    Status: $(cat $rga_dev/status 2>/dev/null || echo 'unknown')"
        ls -la $rga_dev/ | grep -E "(dev|uevent)" | head -5
    fi
done
echo ""

# 5. Check RGA library
echo "[5] RGA Libraries:"
ls -l /usr/lib/librga* 2>/dev/null || echo "  No librga found in /usr/lib"
ls -l /usr/lib/aarch64-linux-gnu/librga* 2>/dev/null || echo "  No librga found in /usr/lib/aarch64-linux-gnu"
ldconfig -p | grep rga || echo "  No RGA in ldconfig cache"
echo ""

# 6. Check RGA headers
echo "[6] RGA Headers:"
ls -l /usr/include/rga/ 2>/dev/null || echo "  No RGA headers in /usr/include/rga"
echo ""

# 7. Try to create device node if missing
echo "[7] Device Node Creation:"
if [ ! -e "/dev/rga" ]; then
    echo "  /dev/rga not found, checking for major/minor..."

    # Check sysfs for dev info
    for rga_dev in /sys/devices/platform/27920000.rga /sys/devices/platform/27930000.rga; do
        if [ -f "$rga_dev/dev" ]; then
            dev_num=$(cat $rga_dev/dev)
            echo "  Found device number: $dev_num"
            major=$(echo $dev_num | cut -d: -f1)
            minor=$(echo $dev_num | cut -d: -f2)
            echo "  Creating /dev/rga with mknod (major=$major, minor=$minor)..."
            sudo mknod /dev/rga c $major $minor 2>/dev/null && echo "  ✓ Created" || echo "  ✗ Failed"
            sudo chmod 666 /dev/rga 2>/dev/null
            break
        fi
    done
else
    echo "  ✓ /dev/rga exists"
    ls -l /dev/rga
fi
echo ""

# 8. Check permissions
echo "[8] Permissions:"
if [ -e "/dev/rga" ]; then
    ls -l /dev/rga
    echo "  Current user: $(whoami)"
    echo "  Groups: $(groups)"

    if [ ! -r "/dev/rga" ] || [ ! -w "/dev/rga" ]; then
        echo "  ⚠ No read/write permission, fixing..."
        sudo chmod 666 /dev/rga
    fi
fi
echo ""

# 9. Test RGA with a simple program
echo "[9] Testing RGA API:"
if [ -f "cpp_fast_targeting/build/rkfast_target" ]; then
    echo "  Running quick RGA test..."
    cd cpp_fast_targeting/build

    # Create a minimal test
    cat > /tmp/test_rga.cpp << 'EOFCPP'
#include <iostream>
#include <rga/RgaApi.h>
#include <rga/im2d.h>

int main() {
    std::cout << "Testing RGA init..." << std::endl;
    int ret = c_RkRgaInit();
    std::cout << "c_RkRgaInit() returned: " << ret << std::endl;

    if (ret == 0 || ret == IM_STATUS_SUCCESS) {
        std::cout << "✓ RGA initialized successfully!" << std::endl;

        // Try to query version
        const char* version = querystring(RGA_VERSION);
        if (version) {
            std::cout << "RGA Version: " << version << std::endl;
        }

        c_RkRgaDeInit();
        return 0;
    } else {
        std::cout << "✗ RGA init failed" << std::endl;
        return 1;
    }
}
EOFCPP

    g++ /tmp/test_rga.cpp -o /tmp/test_rga -lrga 2>/dev/null
    if [ -f "/tmp/test_rga" ]; then
        /tmp/test_rga
        rm /tmp/test_rga
    else
        echo "  Could not compile test program"
    fi
    cd ../..
fi
echo ""

echo "====================================="
echo "Diagnostic Complete"
echo "====================================="
echo ""
echo "Summary:"
echo "  RGA Devices: $(ls -d /sys/devices/platform/*rga 2>/dev/null | wc -l)"
echo "  Device Node: $([ -e /dev/rga ] && echo '✓ exists' || echo '✗ missing')"
echo "  RGA Library: $([ -f /usr/lib/librga.so ] && echo '✓ installed' || echo '✗ not found')"
echo ""
