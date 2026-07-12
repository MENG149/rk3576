# RK3576 C++ Project Cleanup Implementation Plan

## Scope Guard

- Work only in the local repository.
- Preserve the verified FP16 model and all active C++ recognition behavior.
- Keep the root README as the only Markdown file outside docs.

## Tasks

1. Remove the Python package, Python tests, Python entry points, requirements, and Python-only shell scripts.
2. Remove caches, CMake build trees, local backups, duplicate/invalid models, download logs, and backup files.
3. Remove the unreferenced rga_preprocess_v2.cpp experiment and obsolete duplicate build/launch scripts.
4. Move current documents into docs/guides, docs/reports, or docs/reference and preserve retired Python/INT8 guides under docs/archive.
5. Rewrite the root README for the current C++ FP16 workflow.
6. Update retained build scripts to reference best_n_stage1_512_fp16.rknn.
7. Restore C++-appropriate ignore rules for generated output and model uniqueness.

## Verification

- No tracked Python source, Python requirement, or Python-only runtime script remains.
- Active files do not reference removed model names or Python entry points.
- Exactly one RKNN model is tracked and present.
- Retained shell scripts pass bash syntax checks when Bash is available.
- C++ source files used by the runtime are unchanged.
- Git status contains only the intended cleanup.
