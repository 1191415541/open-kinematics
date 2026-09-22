# 09 终局命令逐条记录（EPIC 终局清单）

## T1  exit=0
```
uv run python packages/suspension_multibody/scripts/build_axle_native.py
```
结果: C:\杂件\open-kinematics\packages\suspension_multibody\src\suspension_multibody\native\suspension_kernel.dll 

## T2  exit=0
```
uv run python packages/suspension_kernel/scripts/check_module_layering.py --strict
```
结果:  OK: layering matches the recorded baseline 

## T3  exit=0
```
uv run --package suspension-kernel pytest packages/suspension_kernel/tests -q
```
结果: ...............                                                          [100%] 15 passed in 0.34s 

## T4  exit=0
```
uv run --package suspension-contracts pytest packages/suspension_contracts/tests -q
```
结果: ......................                                                   [100%] 22 passed in 0.11s 

## T5  exit=0
```
uv run --package suspension-multibody pytest packages/suspension_multibody/tests -q
```
结果: ................................x................................        [100%] 737 passed, 47 skipped, 1 xfailed in 327.76s (0:05:27) 

## T6  exit=0
```
uv run --all-packages ruff check .
```
结果: All checks passed! 

## T7  exit=0
```
uv run --all-packages ty check .
```
结果: WARN ty is pre-release software and not ready for production use. Expect to encounter bugs, missing features, and fatal errors. All checks passed! 

## T8  exit=0
```
uv run python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
```
结果:  OK: dynamic output matches the frozen baseline byte-for-byte 

## T9  exit=0
```
uv run python packages/suspension_multibody/scripts/kc_parity_check.py --check
```
结果: OK: candidate matches the frozen K/C snapshot within tolerance 

## T10  exit=0
```
uv run python packages/suspension_multibody/scripts/case_parity_check.py
```
结果:   comparison         N/A       a per-target gate, not a solve: the kernel never reads a reference OK: 8 families accepted 

## T11  exit=0
```
uv run python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
```
结果:  OK: no unregistered Python boundary violation 

## T12  exit=0
```
git diff --check
```
结果:  

## T13  uv build --package suspension-kernel  exit=0
## T14  uv build --package suspension-multibody  exit=0
