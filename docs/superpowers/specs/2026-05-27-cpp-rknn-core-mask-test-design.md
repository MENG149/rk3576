# C++ RKNN Core Mask Test Design

Date: 2026-05-27

## Goal

Add a minimal C++ runtime switch that can force the RKNN context onto a selected NPU core mask, so board-side benchmark runs can determine whether the current model is using one NPU core, dual-core cooperative execution, or parallel independent contexts.

## Scope

Included:

- Add `--core-mask` to `cpp_fast_targeting`.
- Accept `default`, `auto`, `0`, `1`, `01`, and `all`.
- Call the RKNN C API after `rknn_init` and before tensor query/warmup.
- Print the selected mask during startup.
- Document the exact benchmark commands and interpretation.

Not included:

- RGA preprocessing.
- Multiple C++ RKNN workers.
- Camera pipeline changes.
- Tracker or ROI refine changes.

## Design

`AppConfig` stores a string `core_mask` with default `default`, meaning the runtime keeps the RKNN library's existing default behavior. `auto`, `0`, `1`, `01`, and `all` are mapped to RKNN C API core mask constants. Invalid values fail during command-line parsing.

`RknnDetector::Open` calls a helper immediately after `rknn_init`. This keeps the setting attached to the RKNN context before query, warmup, or inference.

If the board-side `rknn_api.h` lacks `rknn_set_core_mask` or the required core mask constants, the build should fail clearly rather than silently ignoring the user's requested test. `default` remains usable without requiring a core-mask call.

## Test Method

Build the C++ target on RK3576, then run the same command with only `--core-mask` changed:

```bash
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

Compare `rknn=...ms`, `total=...ms`, and `infer_fps=...`.

Expected interpretation:

- If `01` is much faster than `0` and `1`, one inference can use dual-core cooperative execution.
- If `01` is similar to `0` and `1`, this model/runtime is effectively single-core per context.
- If two independent contexts are added later and `0,1` roughly doubles aggregate throughput, the two NPU cores can be used through parallel contexts.
