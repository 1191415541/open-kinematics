"""Check the two central simulation architecture invariants."""

from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path("packages/suspension_multibody/src/suspension_multibody")


def _trees() -> list[tuple[Path, ast.Module]]:
    return [
        (path, ast.parse(path.read_text(encoding="utf-8")))
        for path in ROOT.rglob("*.py")
    ]


def _relative(path: Path) -> str:
    return path.relative_to(ROOT).as_posix()


def _decode_definitions(trees: list[tuple[Path, ast.Module]]) -> list[str]:
    result: list[str] = []
    for path, tree in trees:
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == "decode_result":
                result.append(_relative(path))
    return result


def _native_calls(trees: list[tuple[Path, ast.Module]]) -> list[tuple[str, int]]:
    result: list[tuple[str, int]] = []
    for path, tree in trees:
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            direct = isinstance(node.func, ast.Name) and node.func.id == "run_contract"
            qualified = isinstance(node.func, ast.Attribute) and node.func.attr == "run_contract"
            if direct or qualified:
                result.append((_relative(path), node.lineno))
    return result


def _backend_run_calls(tree: ast.Module) -> list[int]:
    for node in tree.body:
        if not isinstance(node, ast.ClassDef) or node.name != "NativeContractBackend":
            continue
        for member in node.body:
            if not isinstance(member, (ast.FunctionDef, ast.AsyncFunctionDef)) or member.name != "run":
                continue
            return [
                call.lineno
                for call in ast.walk(member)
                if isinstance(call, ast.Call)
                and (
                    (isinstance(call.func, ast.Name) and call.func.id == "run_contract")
                    or (isinstance(call.func, ast.Attribute) and call.func.attr == "run_contract")
                )
            ]
    return []


def main() -> int:
    trees = _trees()
    decode_defs = _decode_definitions(trees)
    assert decode_defs == ["results/decoder.py"], decode_defs

    native_calls = _native_calls(trees)
    assert len(native_calls) == 1, native_calls
    assert native_calls[0][0] == "simulation/backend.py", native_calls

    backend_tree = next(
        tree for path, tree in trees if _relative(path) == "simulation/backend.py"
    )
    backend_calls = _backend_run_calls(backend_tree)
    assert len(backend_calls) == 1, backend_calls
    assert native_calls[0][1] == backend_calls[0], (native_calls, backend_calls)

    print("decode_result=results/decoder.py")
    print(f"native_submission=simulation/backend.py:{backend_calls[0]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
