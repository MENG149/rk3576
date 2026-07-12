# RK3576 C++ Project Cleanup Design

## Goal

Keep one runnable C++ RKNN path. Remove the retired Python runtime and local artifacts while preserving the verified FP16 model, detection code, thresholds, and board runtime parameters.

## Structure

Keep `README.md` at the repository root. Place every other Markdown document under `docs/`, grouped into `guides/`, `reports/`, `reference/`, and `superpowers/specs/`.

## Removal Scope

- Python package, tests, entry points, requirements, and Python-only scripts.
- Python/CMake caches and the local Codex backup.
- Duplicate or invalid models, logs, partial downloads, and backup files.
- Unreferenced experimental source files.
- Retired Python/INT8 and duplicate guides move to docs/archive so they cannot be confused with current instructions; unique document content is preserved.

## Corrections

- Make the root README the single C++ build and run guide.
- Point retained scripts to `models/best_n_stage1_512_fp16.rknn`.
- Ignore generated artifacts while allowing only the verified model.
- Do not modify recognition code or parameters.

## Verification

Confirm no active file references Python or removed models, Git tracks exactly one RKNN model, shell scripts pass syntax checks, and the final tree has no build cache or local backup. The prior board build, unit tests, and 207-image zero-difference result remain valid because runtime C++ code is unchanged.

## Non-goals

Do not retrain or convert the model, change recognition behavior, modify the board copy, or overwrite remote history.
