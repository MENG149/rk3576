#!/usr/bin/env bash
#
# benchmark.sh - Performance benchmark and comparison script for rkfast_target
# Usage:
#   ./benchmark.sh              Run benchmark and compare against baseline
#   ./benchmark.sh --baseline   Record current performance as baseline
#
# Target: RK3576 (Linux aarch64)

set -euo pipefail

BINARY="./build/rkfast_target"
DURATION=30
BASELINE_FILE=".benchmark_baseline"
REPORT_FILE="benchmark_report.md"
TMPDIR="${TMPDIR:-/tmp}"
RUN_LOG="$TMPDIR/benchmark_$$.log"

# Colors for terminal output (optional)
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

usage() {
    cat <<EOF
Usage: $0 [--baseline]

Options:
  --baseline    Record current run as the baseline for future comparisons
  -h, --help    Show this help message

Environment:
  DURATION      Benchmark duration in seconds (default: 30)
  BINARY        Path to rkfast_target binary (default: ./build/rkfast_target)
EOF
}

log_info()  { printf "${GREEN}[INFO]${NC}  %s\n" "$*"; }
log_warn()  { printf "${YELLOW}[WARN]${NC}  %s\n" "$*"; }
log_error() { printf "${RED}[ERROR]${NC} %s\n" "$*"; }

# Validate binary exists and is executable
check_binary() {
    if [[ ! -f "$BINARY" ]]; then
        log_error "Binary not found: $BINARY"
        log_info "Build it first, e.g.:"
        log_info "  mkdir -p build && cd build && cmake .. && make"
        exit 1
    fi

    if [[ ! -x "$BINARY" ]]; then
        log_warn "Binary is not executable. Attempting chmod +x ..."
        chmod +x "$BINARY" || {
            log_error "Failed to make $BINARY executable"
            exit 1
        }
    fi
}

# Run the benchmark and capture stats output
run_benchmark() {
    local label="$1"
    log_info "Starting benchmark run: $label"
    log_info "Command: $BINARY --no-window"
    log_info "Duration: ${DURATION}s"

    # Run binary with timeout, capturing stdout+stderr
    # rkfast_target stats typically print lines like:
    #   frame_fps=30.0 infer_fps=30.0 pre=2.1 rknn=8.5 post=1.2 select=0.3 total=12.1
    timeout "$DURATION" "$BINARY" --no-window > "$RUN_LOG" 2>&1 || {
        local rc=$?
        if [[ $rc -eq 124 ]]; then
            log_info "Benchmark completed (timeout after ${DURATION}s)"
        else
            log_warn "Binary exited with code $rc (may be normal for --no-window)"
        fi
    }

    if [[ ! -s "$RUN_LOG" ]]; then
        log_error "No output captured from binary"
        exit 1
    fi
}

# Parse stats from log and compute averages
# Expected format (space or comma separated, or key=value pairs):
#   frame_fps=30.0 infer_fps=30.0 pre=2.1 rknn=8.5 post=1.2 select=0.3 total=12.1
parse_stats() {
    local logfile="$1"
    local -n _out="$2"

    local keys=("frame_fps" "infer_fps" "pre" "rknn" "post" "select" "total")
    local sums=()
    local counts=()

    for _ in "${keys[@]}"; do
        sums+=("0")
        counts+=("0")
    done

    while IFS= read -r line; do
        # Try to extract key=value pairs from the line
        for i in "${!keys[@]}"; do
            local key="${keys[$i]}"
            # Match key=value with optional surrounding whitespace/punctuation
            if [[ "$line" =~ (^|[[:space:]]|,|;)${key}=([0-9]+\.?[0-9]*) ]]; then
                local val="${BASH_REMATCH[2]}"
                sums[$i]=$(awk "BEGIN {print ${sums[$i]} + $val}")
                counts[$i]=$((${counts[$i]} + 1))
            fi
        done
    done < <(grep -E "(frame_fps|infer_fps|pre|rknn|post|select|total)=" "$logfile" 2>/dev/null || true)

    # Compute averages and store in associative array
    for i in "${!keys[@]}"; do
        local key="${keys[$i]}"
        if [[ ${counts[$i]} -gt 0 ]]; then
            local avg
            avg=$(awk "BEGIN {printf \"%.3f\", ${sums[$i]} / ${counts[$i]}}")
            _out["$key"]="$avg"
            _out["${key}_count"]="${counts[$i]}"
        else
            _out["$key"]="N/A"
            _out["${key}_count"]="0"
        fi
    done
}

# Print stats in a readable table
print_stats() {
    local -n _stats="$1"
    local label="$2"

    printf "\n=== %s ===\n" "$label"
    printf "%-12s %12s %12s\n" "Metric" "Average" "Samples"
    printf "%s\n" "-------------------------------------------"
    for key in frame_fps infer_fps pre rknn post select total; do
        printf "%-12s %12s %12s\n" "$key" "${_stats[$key]:-N/A}" "${_stats[${key}_count]:-0}"
    done
}

# Save stats to baseline file
save_baseline() {
    local -n _stats="$1"
    {
        echo "# Baseline recorded: $(date -Iseconds)"
        for key in frame_fps infer_fps pre rknn post select total; do
            echo "${key}=${_stats[$key]}"
            echo "${key}_count=${_stats[${key}_count]}"
        done
    } > "$BASELINE_FILE"
    log_info "Baseline saved to $BASELINE_FILE"
}

# Load baseline stats from file
load_baseline() {
    local -n _stats="$1"
    if [[ ! -f "$BASELINE_FILE" ]]; then
        return 1
    fi
    while IFS='=' read -r key val; do
        [[ "$key" =~ ^# ]] && continue
        [[ -z "$key" ]] && continue
        _stats["$key"]="$val"
    done < "$BASELINE_FILE"
    return 0
}

# Compute percent change: ((new - old) / old) * 100
pct_change() {
    local old="$1"
    local new="$2"
    awk -v o="$old" -v n="$new" 'BEGIN {
        if (o == 0) { print "inf"; exit }
        printf "%.2f", ((n - o) / o) * 100
    }'
}

# Generate markdown report
generate_report() {
    local -n _current="$1"
    local -n _baseline="$2"
    local mode="$3"  # "compare" or "baseline"

    local timestamp
    timestamp=$(date -Iseconds)

    {
        echo "# RK3576 RKNN Performance Benchmark Report"
        echo ""
        echo "- **Timestamp:** $timestamp"
        echo "- **Duration:** ${DURATION}s"
        echo "- **Binary:** \`$BINARY\`"
        echo ""

        if [[ "$mode" == "baseline" ]]; then
            echo "> This run was recorded as the **baseline**."
            echo ""
        fi

        echo "## Current Run"
        echo ""
        echo "| Metric | Average | Samples |"
        echo "|--------|--------:|--------:|"
        for key in frame_fps infer_fps pre rknn post select total; do
            echo "| $key | ${_current[$key]:-N/A} | ${_current[${key}_count]:-0} |"
        done
        echo ""

        if [[ "$mode" == "compare" ]]; then
            echo "## Baseline Comparison"
            echo ""
            echo "| Metric | Baseline | Current | Change |"
            echo "|--------|---------:|--------:|-------:|"
            for key in frame_fps infer_fps pre rknn post select total; do
                local b="${_baseline[$key]:-N/A}"
                local c="${_current[$key]:-N/A}"
                local change="-"
                if [[ "$b" != "N/A" && "$c" != "N/A" ]]; then
                    # For latency metrics (pre, rknn, post, select, total), lower is better
                    # For FPS metrics (frame_fps, infer_fps), higher is better
                    local pct
                    pct=$(pct_change "$b" "$c")
                    if [[ "$key" == "frame_fps" || "$key" == "infer_fps" ]]; then
                        if (( $(echo "$pct > 0" | bc -l) )); then
                            change="+$pct% :arrow_up:"
                        elif (( $(echo "$pct < 0" | bc -l) )); then
                            change="$pct% :arrow_down:"
                        else
                            change="$pct% :heavy_minus_sign:"
                        fi
                    else
                        if (( $(echo "$pct < 0" | bc -l) )); then
                            change="$pct% :arrow_down: (better)"
                        elif (( $(echo "$pct > 0" | bc -l) )); then
                            change="+$pct% :arrow_up: (worse)"
                        else
                            change="$pct% :heavy_minus_sign:"
                        fi
                    fi
                fi
                echo "| $key | $b | $c | $change |"
            done
            echo ""
            echo "### Legend"
            echo "- **FPS metrics** (frame_fps, infer_fps): :arrow_up: = faster, :arrow_down: = slower"
            echo "- **Latency metrics** (pre, rknn, post, select, total): :arrow_down: = faster/better, :arrow_up: = slower/worse"
            echo ""
        fi

        echo "## Raw Log Snippet"
        echo ""
        echo "\`\`\`"
        tail -n 20 "$RUN_LOG"
        echo "\`\`\`"
        echo ""
    } > "$REPORT_FILE"

    log_info "Report written to $REPORT_FILE"
}

# Main entrypoint
main() {
    local mode="compare"

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --baseline)
                mode="baseline"
                shift
                ;;
            -h|--help)
                usage
                exit 0
                ;;
            *)
                log_error "Unknown option: $1"
                usage
                exit 1
                ;;
        esac
    done

    # Allow environment overrides
    BINARY="${BINARY:-./build/rkfast_target}"
    DURATION="${DURATION:-30}"

    check_binary

    declare -A current_stats
    declare -A baseline_stats

    run_benchmark "$mode"
    parse_stats "$RUN_LOG" current_stats
    print_stats current_stats "Current Run"

    if [[ "$mode" == "baseline" ]]; then
        save_baseline current_stats
        generate_report current_stats baseline_stats "baseline"
        log_info "Baseline recorded. Future runs will compare against these numbers."
    else
        if load_baseline baseline_stats; then
            print_stats baseline_stats "Baseline"
            generate_report current_stats baseline_stats "compare"
        else
            log_warn "No baseline found. Run with --baseline to create one."
            generate_report current_stats baseline_stats "baseline"
        fi
    fi

    # Cleanup
    rm -f "$RUN_LOG"
}

main "$@"
