"""
The kernel boundary: one container of model, one of case, one result back.

This is the thin shell the architecture asks for -- it packs two documents,
calls the single ABI entry point, and unpacks what comes back.  It holds no
physics: the case layer inside the kernel expands grids and load paths, the
solver computes states, and everything this module does with the answer is
turning a blob into arrays.

The ctypes mirror in ``axle_dynamics.native`` stays alongside it until the
callers that were compiled against it are migrated; both load the same shared
library, so there is exactly one kernel.

The concrete block layout a caller gets back: ``body_state`` is
``[sample, body, 19]`` with ``[x, y, z, qw, qx, qy, qz, v, omega, ...]``, and
``blocks`` is keyed by block name so a caller never indexes the blob itself.
"""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
from typing import Any

import numpy as np
from suspension_contracts import pack_container, unpack_container

from ..axle_dynamics.native import _load_library

__all__ = ["KernelContractError", "ContractRun", "contract_version", "run_contract"]

_DTYPE = {"float64": np.float64, "int32": np.int32, "uint8": np.uint8}


class KernelContractError(RuntimeError):
    """
    Raised when the kernel refuses a contract payload or fails a run.

    A *failed run* still has a document: the kernel writes the samples that did
    converge and, in the manifest, where it stopped.  That document is attached
    as ``partial_run`` so a caller can report the failure with its evidence
    instead of only its message.
    """

    def __init__(self, message: str, *, partial_run: "ContractRun | None" = None) -> None:
        super().__init__(message)
        self.partial_run = partial_run


@dataclass(frozen=True)
class ContractRun:
    """A parsed result document and its blocks, keyed by name."""

    document: dict[str, Any]
    blocks: dict[str, np.ndarray]

    def block(self, name: str) -> np.ndarray:
        """Return a named block, or raise if the kernel did not emit it."""
        try:
            return self.blocks[name]
        except KeyError as error:  # pragma: no cover - defensive
            raise KernelContractError(f"result has no block {name!r}") from error

    @property
    def status(self) -> str:
        return str(self.document.get("status", "failed"))

    def cases(self) -> list[dict[str, Any]]:
        """Return the expanded case list the kernel reported, in run order."""
        manifest = self.document.get("manifest", {})
        return list(manifest.get("cases", []))


def _library() -> ctypes.CDLL:
    library = _load_library()
    if not hasattr(library, "suspension_kernel_run"):
        raise KernelContractError(
            "the loaded kernel has no suspension_kernel_run entry point; "
            "rebuild it with packages/suspension_multibody/scripts/build_axle_native.py"
        )
    library.suspension_kernel_run.argtypes = [
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.POINTER(ctypes.c_size_t),
        ctypes.c_char_p,
        ctypes.c_size_t,
    ]
    library.suspension_kernel_run.restype = ctypes.c_int32
    library.suspension_kernel_contract_version.restype = ctypes.c_int32
    return library


def contract_version() -> int:
    """Return the contract version the loaded kernel implements."""
    return int(_library().suspension_kernel_contract_version())


def _invoke(library: ctypes.CDLL, model: bytes, case: bytes) -> bytes:
    model_buffer = (ctypes.c_uint8 * len(model)).from_buffer_copy(model)
    case_buffer = (ctypes.c_uint8 * len(case)).from_buffer_copy(case)
    capacity = 1 << 20
    while True:
        result = (ctypes.c_uint8 * capacity)()
        length = ctypes.c_size_t(capacity)
        error = ctypes.create_string_buffer(4096)
        status = library.suspension_kernel_run(
            model_buffer,
            len(model),
            case_buffer,
            len(case),
            result,
            ctypes.byref(length),
            error,
            len(error),
        )
        if status == 11:
            # The kernel reports the size it needs; grow once and retry rather
            # than guessing a capacity that happens to be large enough.
            capacity = int(length.value)
            continue
        if status != 0:
            message = error.value.decode("utf-8", errors="replace")
            raise KernelContractError(message or f"kernel returned status {status}")
        return bytes(result[: length.value])


def _read_blocks(document: dict[str, Any], blob: bytes) -> dict[str, np.ndarray]:
    blocks: dict[str, np.ndarray] = {}
    for descriptor in document.get("blocks", []):
        dtype = _DTYPE[str(descriptor["dtype"])]
        offset = int(descriptor["offset"])
        length = int(descriptor["length"])
        shape = tuple(int(extent) for extent in descriptor["shape"])
        values = np.frombuffer(blob, dtype=dtype, count=length // dtype().itemsize, offset=offset)
        blocks[str(descriptor["name"])] = values.reshape(shape).copy()
    return blocks


def run_contract(
    model_document: dict[str, Any],
    case_document: dict[str, Any],
    *,
    model_payload: bytes | None = None,
    case_payload: bytes | None = None,
) -> ContractRun:
    """
    Run one model document against one case document.

    ``model_payload`` and ``case_payload`` let a caller that already built the
    container -- one whose document describes tables in the blob, so it cannot
    be packed from the document alone -- hand the finished payload over instead.
    """
    payload = _invoke(
        _library(),
        model_payload if model_payload is not None else pack_container(model_document),
        case_payload if case_payload is not None else pack_container(case_document),
    )
    document, blob = unpack_container(payload)
    if document.get("status") != "success":
        manifest = document.get("manifest", {})
        message = str(
            manifest.get("failure_message")
            or f"kernel reported {document.get('status')}"
        )
        raise KernelContractError(
            message,
            partial_run=ContractRun(document=document, blocks=_read_blocks(document, blob)),
        )
    return ContractRun(document=document, blocks=_read_blocks(document, blob))
