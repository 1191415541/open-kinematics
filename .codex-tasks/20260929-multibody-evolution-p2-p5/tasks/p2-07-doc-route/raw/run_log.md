# p2-07 运行日志（命令与实测退出码）

全部在仓库根目录执行，Windows / Git Bash。每条命令后是其**实测**退出码。

## 1. 重建内核（改了 C++，必须先重编）

```
$ uv run --no-sync python packages/suspension_kernel/scripts/build_suspension_kernel.py
E:\杂件\open-kinematics\packages\suspension_kernel\src\suspension_kernel\native\suspension_kernel.dll
exit=0

$ uv run --no-sync python packages/suspension_multibody/scripts/build_axle_native.py
E:\杂件\open-kinematics\packages\suspension_multibody\src\suspension_multibody\native\suspension_kernel.dll
exit=0
```

## 2. 改前的拒绝复核（改造前落盘的那次不再重跑；这里只复核改造后的行为）

```
$ uv run --no-sync python .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-07-doc-route/raw/document_route_probe.py
KernelContractError: case document: case document has no time object
exit=0
```

`exit=0` 是探针脚本自身的语义（它 catch 异常后 `print`，不 re-raise）。
关键点：**报错不再是** `model document: element "torque" has unsupported type "rotational_torque"`，
而是下一层发现那份最小探针的 case 文档缺 `time`——模型文档那一步已经通过了。

## 3. C++ 自测

```
$ packages/suspension_kernel/build/Release/mb_contract_selftest.exe
mb_contract selftest: OK (45 checks)
exit=0
```

```
$ uv run --no-sync pytest packages/suspension_kernel/tests/test_registry_consistency.py -q -p no:cacheprovider
12 passed in 1.11s
exit=0
```

## 4. 验收测试

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/cases/test_rotational_torque_document.py -q -p no:cacheprovider
7 passed in 0.28s
exit=0
```

## 5. 两个必须各自独立跑的套件

```
$ uv run --no-sync pytest packages/suspension_contracts/tests -q -p no:cacheprovider
32 passed in 0.10s
exit=0

$ uv run --no-sync pytest packages/suspension_kernel/tests -q -p no:cacheprovider
41 passed in 14.66s
exit=0
```

（分别单独调用；合并会改 rootdir 并破坏 import。）

## 6. 产物

```
$ uv run --no-sync python .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-07-doc-route/raw/document_route_two_run.py
driven   final: with=1.499999999999999 without=3.0 difference=1.500000000000001
reaction final: with=1.5000000000000002 without=0.0 difference=1.5000000000000002
tolerance: 0.5
omitted reference pose is byte-identical to the explicit identity: True
exit=0

$ uv run --no-sync python .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p2-07-doc-route/raw/existing_family_probe.py
status: success
exit=0
```

## 7. 冻结基线与 tests/data

```
$ uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
artifacts hashed : 26
combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
OK: dynamic output matches the frozen baseline byte-for-byte
exit=0

$ git status --short -- packages/suspension_multibody/tests/data/
（空输出）
exit=0
```

## 8. 未改 ABI 的自证

```
$ git diff --stat -- packages/suspension_kernel/cpp/include/mb_config/version.hpp \
                  packages/suspension_multibody/src/suspension_multibody/kernel/native.py
 .../suspension_kernel/cpp/include/mb_config/version.hpp     | 13 +++++++++++--
 .../src/suspension_multibody/kernel/native.py               |  4 ++--
 2 files changed, 13 insertions(+), 4 deletions(-)
exit=0
```

**这条命令的输出不为空**，因为它比较的是工作区与 `HEAD`，而这两个文件在本工作区里
带着 **p2-02 的未提交改动**（ABI 16/31 → 17/32 及其 Python 镜像；diff 正文自带
`(p2-02, 2026-10-01)` 的注释）。用「diff 是否为空」当自证在这里是无效判据，改用三条互相独立的证据：

```
$ stat -c '%y %n' packages/suspension_kernel/cpp/include/mb_config/version.hpp \
                  packages/suspension_multibody/src/suspension_multibody/kernel/native.py
2026-10-01 08:19:29.932422300 +0800  .../mb_config/version.hpp
2026-10-01 08:19:29.968999500 +0800  .../kernel/native.py
exit=0

$ sha256sum packages/suspension_kernel/cpp/include/mb_config/version.hpp \
            packages/suspension_multibody/src/suspension_multibody/kernel/native.py
5ec0e968fc78570bd3d02e7db7d518e3e573e6f14849d4b73d3d059edbaf2094  .../mb_config/version.hpp
c447527f8e43e0f47b08f6800628121a01a90b106720792737744b4345f690db  .../kernel/native.py
exit=0
```

1. **本行的写操作从未指向这两个路径**：全部 `Edit`/`Write` 的目标只有
   `contract_registry.cpp`、`contract_model.cpp`、`kernel_contract_run.cpp`、
   `mb_cases/functions.hpp`、`contract_selftest.cpp`、`multibody_model.schema.json`、
   新测试与本次任务目录的 `raw/` / `PROGRESS.md`。
2. **mtime**：两个时刻相隔 36 毫秒、都停在 `08:19:29`——本行会话开始之前；
   本行自己的 C++ 改动落在 `09:02 - 09:04`。
3. **内容**：两者当前仍是 p2-02 落的 `17 / 32`（`version.hpp`）与 `17 / 32 / 1`（`native.py`）。

## 9. 静态检查

```
$ uv run --no-sync ruff check . --output-format=concise
All checks passed!
exit=0

$ uv run --no-sync ty check .
All checks passed!
exit=0
```

`ruff --fix` 修掉的 3 条全在本行写范围内的两个文件：

- `raw/document_route_probe.py`（首次落盘时的 `import json, sys`）：拆行并删掉未用的 `json`；
- `packages/suspension_multibody/tests/cases/test_rotational_torque_document.py`：import 分组。

其余 3 条（`D401`）按上下文改写 docstring 首行，未改行为。盘面上其它 `raw/` 探针脚本未被触碰。

## 10. 回归面（额外跑的套件，非规格强制）

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/cases -q -p no:cacheprovider
107 passed in 45.04s
exit=0

$ uv run --no-sync pytest packages/suspension_multibody/tests/cases/kc_quasi_static -q -p no:cacheprovider
34 passed in 3.14s
exit=0

$ uv run --no-sync pytest packages/suspension_multibody/tests/axle_dynamics \
                       packages/suspension_multibody/tests/subsystems \
                       packages/suspension_multibody/tests/modeling -q -p no:cacheprovider
376 passed in 15.19s
exit=0
```

## 11. 未执行 / 未能执行

- 未执行 `dynamic_hash_sentinel.py --record`（禁止重录基线）。
- 未跑 GUI / Adams 相关的 `manual`、`adams` 标记用例（上游 pyproject 默认 `-m 'not manual'`，
  且不在本行回归面内）。
- 未改任何写范围之外的文件：`mb_config/version.hpp`、`mb_input/types.hpp`、`kernel/native.py`、
  `build_model.cpp`、`element/anti_roll.cpp`、`cases/vehicle_dynamic.py`、`EPIC.md`、`SUBTASKS.csv`、
  其它 `tasks/` 目录、任何冻结基线，均未触碰。
