#!/bin/bash
# =============================================================================
# RK3576 RKNN Targeting — Build & Verification Script
# =============================================================================
# Run this directly on the RK3576 board after ensuring dependencies are
# installed (librknnrt, librga, OpenCV, pkg-config).
#
# Usage:
#   chmod +x build_and_verify.sh
#   ./build_and_verify.sh
# =============================================================================

set -euo pipefail

# --- Configuration -----------------------------------------------------------
BUILD_DIR="build"
BINARY_NAME="rkfast_target"
CMAKE_LOG="${BUILD_DIR}/cmake_configure.log"

# Colours for terminal output (safe if TERM unset)
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Colour

# Track overall status
OVERALL_STATUS=0

# Helper: print section header
section() {
  echo ""
  echo -e "${CYAN}=== $1 ===${NC}"
}

# Helper: print success / warning / error
ok()   { echo -e "${GREEN}[OK]${NC}   $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
fail() { echo -e "${RED}[FAIL]${NC} $1"; OVERALL_STATUS=1; }

# --- 1. Environment sanity checks -------------------------------------------
section "Environment Checks"

# Detect architecture
ARCH=$(uname -m)
ok "Architecture: ${ARCH}"

if [[ "${ARCH}" != "aarch64" ]]; then
  warn "Not running on aarch64 — binary may not be optimal for RK3576"
fi

# Verify critical tools exist
for tool in cmake make pkg-config; do
  if command -v "${tool}" &>/dev/null; then
    ok "Found $(printf '%-12s' "${tool}:") $(command -v ${tool})"
  else
    fail "Missing required tool: ${tool}"
  fi
done

# Verify NPU / RGA kernel modules (informational)
if [[ -d /dev/dri ]]; then
  ok "DRM devices present"
  ls /dev/dri/render* 2>/dev/null | while read -r dev; do
    echo "       ${dev}"
  done || true
else
  warn "No /dev/dri — RGA may not be available"
fi

# --- 2. Clean previous build -------------------------------------------------
section "Clean Build"

if [[ -d "${BUILD_DIR}" ]]; then
  echo "Removing existing build directory ..."
  rm -rf "${BUILD_DIR}"
fi
mkdir -p "${BUILD_DIR}"
ok "Build directory ready: $(pwd)/${BUILD_DIR}"

# --- 3. CMake configuration (Release, RGA ON) --------------------------------
section "CMake Configuration"

echo "Running cmake -DCMAKE_BUILD_TYPE=Release -DRKFAST_USE_RGA=ON ..."
if cmake -S . -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DRKFAST_USE_RGA=ON \
    -DCMAKE_CXX_FLAGS="-march=armv8.2-a -mtune=cortex-a76" \
    2>&1 | tee "${CMAKE_LOG}"; then
  ok "CMake configuration succeeded"
else
  fail "CMake configuration failed"
  echo ""
  echo "CMake log tail:"
  tail -n 30 "${CMAKE_LOG}" || true
  exit 1
fi

# --- 4. Check RGA detection in cmake output ----------------------------------
section "RGA Detection Check"

if grep -q "RGA found:" "${CMAKE_LOG}"; then
  RGA_LINE=$(grep "RGA found:" "${CMAKE_LOG}")
  ok "${RGA_LINE}"
elif grep -q "RGA requested but not found" "${CMAKE_LOG}"; then
  RGA_WARN=$(grep "RGA requested but not found" "${CMAKE_LOG}")
  fail "${RGA_WARN}"
  echo ""
  echo "Troubleshooting:"
  echo "  1. Install librga-dev / librga package"
  echo "     (e.g. sudo apt install librga-dev  or copy from SDK)"
  echo "  2. Verify headers:  ls /usr/include/rga/"
  echo "  3. Verify library:  ldconfig -p | grep librga"
  echo ""
else
  warn "Could not find explicit RGA message in cmake log — inspect manually"
fi

# --- 5. Build ----------------------------------------------------------------
section "Build"

NPROC=$(nproc)
ok "Parallel jobs: ${NPROC}"

if cmake --build "${BUILD_DIR}" --parallel "${NPROC}" 2>&1 | tee "${BUILD_DIR}/build.log"; then
  ok "Build completed successfully"
else
  fail "Build failed"
  echo ""
  echo "Build log tail:"
  tail -n 40 "${BUILD_DIR}/build.log" || true
  exit 1
fi

# --- 6. Verify binary exists -------------------------------------------------
section "Binary Verification"

BINARY_PATH="${BUILD_DIR}/${BINARY_NAME}"
if [[ -x "${BINARY_PATH}" ]]; then
  ok "Binary exists and is executable: ${BINARY_PATH}"
else
  fail "Binary not found or not executable: ${BINARY_PATH}"
  exit 1
fi

# Print size
BINARY_SIZE=$(stat -c%s "${BINARY_PATH}" 2>/dev/null || stat -f%z "${BINARY_PATH}" 2>/dev/null || echo "?")
if [[ "${BINARY_SIZE}" != "?" ]]; then
  printf "       Size: %'d bytes (%.2f MB)\n" "${BINARY_SIZE}" "$(echo "scale=2; ${BINARY_SIZE}/1024/1024" | bc 2>/dev/null || echo "N/A")"
fi

# --- 7. Check dynamic linking (librga + librknnrt) ---------------------------
section "Library Linkage Check"

# Use readelf if available, fallback to ldd
if command -v readelf &>/dev/null; then
  NEEDED=$(readelf -d "${BINARY_PATH}" | grep NEEDED | awk '{print $5}' | tr -d '[]')
else
  NEEDED=$(ldd "${BINARY_PATH}" 2>/dev/null | awk '{print $1}')
fi

if echo "${NEEDED}" | grep -qE 'librga\.so|librga\.'; then
  ok "Binary links against librga"
else
  fail "Binary does NOT link against librga"
fi

if echo "${NEEDED}" | grep -qE 'librknnrt\.so|librknnrt\.'; then
  ok "Binary links against librknnrt"
else
  fail "Binary does NOT link against librknnrt"
fi

# Show full dependency list for reference
echo ""
echo "Full NEEDED libraries:"
echo "${NEEDED}" | sed 's/^/  /'

# --- 8. Smoke test (--help) --------------------------------------------------
section "Smoke Test"

if "${BINARY_PATH}" --help &>/dev/null; then
  ok "Binary runs and responds to --help"
  echo ""
  "${BINARY_PATH}" --help | sed 's/^/  /'
else
  # Some binaries exit non-zero on --help; check if output was produced
  HELP_OUT=$("${BINARY_PATH}" --help 2>&1 || true)
  if [[ -n "${HELP_OUT}" ]]; then
    ok "Binary produced help output (exit code may be non-zero by design)"
    echo ""
    echo "${HELP_OUT}" | sed 's/^/  /'
  else
    fail "Binary did not respond to --help (no output)"
  fi
fi

# --- 9. Runtime library resolution check -------------------------------------
section "Runtime Library Resolution"

if command -v ldd &>/dev/null; then
  MISSING_LIBS=$(ldd "${BINARY_PATH}" 2>/dev/null | grep 'not found' || true)
  if [[ -z "${MISSING_LIBS}" ]]; then
    ok "All runtime libraries resolved"
  else
    fail "Missing runtime libraries:"
    echo "${MISSING_LIBS}" | sed 's/^/  /'
  fi
else
  warn "ldd not available — skipping runtime resolution check"
fi

# --- 10. Summary -------------------------------------------------------------
section "Build Summary"

if [[ ${OVERALL_STATUS} -eq 0 ]]; then
  echo -e "${GREEN}BUILD SUCCESS${NC}"
  echo ""
  echo "  Binary:     $(realpath "${BINARY_PATH}")"
  echo "  Build type: Release"
  echo "  RGA:        Enabled"
  echo ""
  echo "Next steps:"
  echo "  1. Verify the model exists: ../models/best_n_stage1_512_fp16.rknn"
  echo "  2. Run inference:"
  echo ""
  echo "     ${BINARY_PATH} --model ../models/best_n_stage1_512_fp16.rknn \\"
  echo "       --camera /dev/video73 --width 640 --height 480 --fps 60 \\"
  echo "       --fourcc MJPG --no-window"
  echo ""
  exit 0
else
  echo -e "${RED}BUILD FAILED${NC} — see [FAIL] messages above"
  exit 1
fi
