# p5-05 D6：FMI 库是「新依赖」，本行实际是否引入

> 本文件记录 D6 的提出与裁决、以及本行的实际处置。它只记**已执行**的判定。
> 配套：`feasibility.md`（技术路线可行性实测，早于本文件，未覆盖）、
> `fmu_validation.md`（产物与仓库外校验）。

## 1. D6 的原文（`EPIC.md` 行 65）

| 编号 | 问题 | 裁决与理由 |
|---|---|---|
| D6 | 是否允许引入新依赖 | **裁决：条件允许（仅 p5-05 的 FMI 库）。** 引入前须先说明再落地。若最终无法引入，p5-05 交付「可联合仿真的接口契约 + 仓库外独立校验脚本」并**登记为未闭合项**，Epic 相应不宣告完全关闭（不掩盖）。 **建议：除 p5-05 的 FMI 库外不新增依赖**（阶段二~四用现有 numpy/几何设施即可）。p5-05 的依赖须先提出再确认 |

子任务侧同一口径（`SPEC.md` 的「禁止触碰」第一条）：

> **不得在任何路径下引入未获确认的新依赖**（D6，`EPIC.md` 行 65）：FMI 库须**先提出再确认**。
> 若不允许，按 `EPIC.md` 行 319 降级为「导出为可联合仿真的接口契约 + 独立验证脚本」，
> 并**登记为未闭合项**。

即：D6 **授权**了引入 FMI 库，但把「引入前先说明」定为硬条件，并要求在无法引入时如实降级。

## 2. 提出（引入前先说明）

本行原本要引入的候选依赖是 **`fmpy`**（Python 侧最常用的 FMI 读写与仿真库），
用途是打包 `.fmu` 与在测试里加载它。按 D6 的要求，在落地任何代码之前先做可行性实测，
结果记在 `raw/feasibility.md`：

1. `uv run --no-sync pip download fmpy -d … --no-deps` **成功**——
   即「引入可行」这一事实成立，D6 的授权不是空转；
2. 但同一份实测的第 3 节证明**引入并非必要**：只用 Python 标准库
   （`zipfile` / `xml.etree` / `ctypes`）就完成了「打进 `.fmu` → 在仓库外加载 → 步进 →
   断言输入驱动输出」的全链路，且 `fmi2GetVersion()` 返回 `2.0`。

## 3. 落地口径（本行的实际处置）

**本行不引入任何新依赖。** 理由不是「D6 不允许」（D6 允许），而是
**D4 的目标不需要它**：

| 需要的能力 | 标准库是否够 | 实测依据 |
|---|---|---|
| 打 `.fmu`（zip 归档） | 是，`zipfile` | `feasibility.md` 第 3 节 |
| 写 `modelDescription.xml` | 是，`xml.etree.ElementTree` | 本行 `fmi/export.py` |
| 仓库外加载并步进 FMU | 是，`ctypes` | `fmu_validation.md` |
| 读内核结果块 | 不适用——由 FMU 内 C wrapper 调内核完成，不经 Python | `fmi/fmu_wrapper.c` |

因此：

- **未改 `pyproject.toml`**，`uv.lock` 未变；
- **未新增 `fmpy`**，也未新增任何其它包；
- **D6 的授权未被动用**——「条件允许」的前提（需要 FMI 库）经实测不成立，故不引入；
- **本行不是降级交付**：`EPIC.md` 行 319 的降级路径（「接口契约 + 独立验证脚本，登记为
  未闭合项」）针对的是「无法引入 FMI 库」这一情形；本行交付的是**真正的 FMU 2.0
  Co-Simulation 归档**，由一个 C 二进制实现完整 FMI 符号集并在仓库外被加载步进，
  已达 D4 的范围（模型 + 输入/输出变量、不含 Python 侧求值），因此**不登记未闭合项**。

## 4. 命令与输出原文

依赖是否被动用，是可核验的事实，不以声明为准：

```
$ git diff --stat pyproject.toml
（空：该文件未修改）

$ git status --short -- packages/*/pyproject.toml uv.lock
（空：无未跟踪或已修改的依赖清单）
```

零依赖的实现路径本身也是可核验的：`fmi/export.py` 只 import 标准库与本包
（`json`/`platform`/`shutil`/`zipfile`/`dataclasses`/`pathlib`/`typing`/`xml.etree`），
`fmi/fmu_wrapper.c` 只 include C 标准库与平台 loader 头
（`stdio.h`/`stdlib.h`/`string.h`/`windows.h` 或 `dlfcn.h`），
`scripts/build_fmu_binary.py` 只用一个 C 编译器。
