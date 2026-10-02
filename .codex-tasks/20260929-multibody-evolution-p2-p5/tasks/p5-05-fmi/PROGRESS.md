# PROGRESS：p5-05 FMI 联合仿真导出

> 父 Epic：`.codex-tasks/20260929-multibody-evolution-p2-p5/EPIC.md`；父行：`SUBTASKS.csv` 的 `p5-05`

## Session Start

- **Date**: 2026-10-02
- **Task name**: p5-05-fmi
- **Task dir**: `.codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p5-05-fmi/`
- **Spec**: 见 `SPEC.md`
- **Plan**: 见 `TODO.csv`（4 步，全部 DONE）
- **Environment**: Python 3.12 / uv / pytest；x86_64 MinGW gcc（`-dumpmachine` → `x86_64-w64-mingw32`）；仓库根 `E:\杂件\open-kinematics`
- **Depends on**: `p5-04`（`SUBTASKS.csv` 第 23 行），并受全局前置 `S1`（阶段一 Epic 01–07 全 `DONE`）约束

## Context Recovery Block

- **Current milestone**: 完成（4/4）
- **Current status**: DONE
- **Last completed**: TODO.csv 第 4 步（既有运行路径零影响与门禁复验）
- **Current artifact**: `fmi/`（新包）、`tests/api/test_fmu_export.py`、`scripts/build_fmu_binary.py`；证据 `raw/`
- **Key context**:
  - D4 裁决：**FMU 2.0 Co-Simulation**，只导出模型 + 输入/输出变量，不含 Python 侧求值，不做实时/硬件在环。
  - D6 裁决：FMI 库属新依赖，**条件允许**；实测标准库已足够 → **未引入任何新依赖**。
  - 输入/输出变量来自 case 文档自身 `blobs` 与 p5-03 的信号总线（结果块列）。
- **Known issues**: 无未闭合项（不是降级交付，理由见 `raw/dependency_decision.md` 第 3 节）。
- **Next action**: 父 Epic 由 p5-06 收口。

---

## 交付物

| 文件 | 作用 |
|---|---|
| `packages/suspension_multibody/src/suspension_multibody/fmi/__init__.py` | 包出口 |
| `…/fmi/export.py` | Python 侧：契约容器、变量清单与**绑定**、`modelDescription.xml`、归档 |
| `…/fmi/fmu_wrapper.c` | C 侧：完整 FMI 2.0 Co-Simulation 符号集，调内核求解 |
| `packages/suspension_multibody/scripts/build_fmu_binary.py` | 编译 wrapper（按 target triple 拒绝非 64 位编译器） |
| `packages/suspension_multibody/tests/api/test_fmu_export.py` | 16 个用例 |
| `.gitignore` | `fmi/*.dll|*.dylib|*.so` 为构建产物 |

## 实测结论（关键数字）

- 归档 `rig.fmu` **22435 B**；`fmi2GetVersion()` == `2.0`。
- 变量 **17**（2 input / 15 output），`valueReference` 0..16 稠密。
- 输入绑定 = case `blobs` descriptor 的 `offset` / `length//8`（`0/201`、`1608/201`）。
- **轨迹断言**：输入 `0.0 → 0.9`，纵向滑移时程逐样本最大差 **2.188280**。
- 步进确实换样本：同一次运行内滑移极差 **3.933832**。
- 同 pair 两次导出**逐字节相同**（GUID 由 `canonical_hash` 派生，非随机/非时间戳）。

## 本行触及的两个真缺陷（实测发现，已修）

1. **FMU 内的容器 blob 为空**：导出器原先只写文档不写 blob，内核读该容器直接
   `status 2: case document: role brake_pressure range falls outside the payload blob`。
   根因：`compile_document_pair` 需要的是**容器字节**（`pack_container`），而暴露的
   裸 blob 被直接当容器传入。修：`_document_and_payload` 负责打包，并对「有 descriptor
   无 payload」按名拒绝。
2. **wrapper 的块扫描器方向错**：`block_descriptor()` 的 JSON 键是**字母序**
   （`dtype, length, name, offset, order, shape`），`offset`/`length` 出现在 `name`
   **之前**；从名字向后扫会读到下一个描述符的字段（静默读错块）。修：从名字回退到
   该对象的 `{`，在对象边界内分别取字段；同时按 `shape` 的第二/三 extent 取行数与列宽
   （原先列偏移是对第 0 个样本硬编码的）。

## 未做（如实登记）

- 不做实时/硬件在环承诺（D4 + Non-Goals 行 101）。
- 归档内无 Python 侧求值（D4）。
- FMU 状态序列化与方向导数按「不支持」应答，不假装支持。
- `ASSEMBLY_OUTPUTS` 中不是结果 blob 列的声明（upright 位姿/轮心/诊断计数）**不导出**，
  理由见 `fmi/export.py` 模块 docstring。

## Final Summary

**DONE。** `TODO.csv` 4 行全部 `DONE`，`completed_at = 2026-10-02`，`retry_count = 0`。

逐条对 `SPEC.md` 的 Done-When：

- [x] 按 D4 版本与范围导出 FMU；产物存在；变量清单（名称 + 方向）逐条列出并判定正确 —— `raw/fmu_validation.md` 第 2 节（17 条，逐条给方向理由）。
- [x] 仓库外独立脚本加载并步进；不 import 本仓库模块；内容、命令、输出原文落盘 —— `raw/fmu_validation.md` 第 3、5 节（`fmi2GetVersion: 2.0`，轨迹断言 2.188280，`EXIT=0`）。
- [x] D6 的依赖提出与裁决已记录；**未引入依赖**（不是降级，见 `raw/dependency_decision.md`）。
- [x] 导出不影响既有运行路径：`tests/api` 75 passed；`kc_baseline` 与 `dynamic_hash_baseline` 逐字节未变 —— `raw/no_regression.md`。
- [x] 未改 ABI（仍 17/32/1）；提交点仍唯一在 `simulation/backend.py`；未重录基线；未新增 skip/xfail —— `raw/gates.txt` 第 4、7、8 节。
- [x] 未做任何实时/硬件在环承诺 —— `modelDescription.xml` 的 `canRunAsynchronously=false`，且文档第 6 节登记。

`Final Validation Command`（`SPEC.md`）：

```
uv run --no-sync pytest packages/suspension_multibody/tests/api -q && test -s …/tasks/p5-05-fmi/raw/fmu_validation.md
```

实跑：**75 passed**，`test -s` 通过（`raw/gates.txt` 第 1 节）。
