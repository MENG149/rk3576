from __future__ import annotations

from typing import Any

import numpy as np

from .config import RuntimeConfig


class RknnYoloRuntime:
    def __init__(self, config: RuntimeConfig, verbose: bool = False) -> None:
        self.config = config
        self.verbose = verbose
        self.rknn: Any = None
        self.rknn_lite_class: Any = None

    def open(self) -> None:
        try:
            from rknnlite.api import RKNNLite
        except ImportError as exc:
            raise RuntimeError(
                "Missing rknn-toolkit-lite2. Install the RK3576 aarch64 wheel on the board."
            ) from exc

        model_path = self.config.model.expanduser().resolve()
        if not model_path.is_file():
            raise FileNotFoundError(f"RKNN model not found: {model_path}")

        self.rknn_lite_class = RKNNLite
        try:
            self.rknn = RKNNLite(verbose=self.verbose)
        except TypeError:
            self.rknn = RKNNLite()

        ret = self.rknn.load_rknn(str(model_path))
        if ret != 0:
            raise RuntimeError(f"load_rknn failed: {ret}")

        core_mask = self._resolve_core_mask(self.config.core_mask)
        try:
            ret = self.rknn.init_runtime() if core_mask is None else self.rknn.init_runtime(core_mask=core_mask)
        except TypeError:
            ret = self.rknn.init_runtime()
        if ret != 0:
            raise RuntimeError(f"init_runtime failed: {ret}")

    def _resolve_core_mask(self, name: str) -> Any:
        normalized = name.lower().replace("-", "_")
        if normalized in ("default", "none"):
            return None

        aliases = {
            "auto": ("NPU_CORE_AUTO",),
            "all": ("NPU_CORE_0_1_2", "NPU_CORE_ALL"),
            "0": ("NPU_CORE_0",),
            "1": ("NPU_CORE_1",),
            "2": ("NPU_CORE_2",),
            "01": ("NPU_CORE_0_1",),
            "02": ("NPU_CORE_0_2",),
            "12": ("NPU_CORE_1_2",),
        }
        for attr in aliases.get(normalized, ()):
            if hasattr(self.rknn_lite_class, attr):
                return getattr(self.rknn_lite_class, attr)
            if hasattr(self.rknn, attr):
                return getattr(self.rknn, attr)
        return None

    def inference(self, input_rgb: np.ndarray) -> list[np.ndarray]:
        if self.rknn is None:
            raise RuntimeError("RKNN runtime is not initialized")
        return self.rknn.inference(inputs=[np.expand_dims(input_rgb, axis=0)])

    def release(self) -> None:
        if self.rknn is not None:
            self.rknn.release()
            self.rknn = None

