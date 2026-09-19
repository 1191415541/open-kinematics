"""
The axle product's error types.

They live here rather than beside the ctypes mirror because the contract route
raises them too: which route reached the kernel is not something a caller should
have to know to catch a failed run.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from .result import DIAGNOSTIC_COLUMNS


class NativeAxleError(RuntimeError):
    """Raised when the native solver rejects or fails a run."""

    def __init__(
        self,
        message: str,
        *,
        status: int,
        partial_result: Any | None = None,
        failure_diagnostics: np.ndarray | None = None,
        failed_sample_index: int | None = None,
        failed_time_s: float | None = None,
    ) -> None:
        super().__init__(message)
        self.status = status
        self.partial_result = partial_result
        self.failure_diagnostics = failure_diagnostics
        self.failed_sample_index = failed_sample_index
        self.failed_time_s = failed_time_s
        self.named_failure_diagnostics = (
            None
            if failure_diagnostics is None
            else {
                name: float(value)
                for name, value in zip(
                    DIAGNOSTIC_COLUMNS, failure_diagnostics
                )
            }
        )
