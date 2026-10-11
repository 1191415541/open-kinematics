# 验证命令与Windows执行

所有命令在仓库根执行，不安装新依赖，不重录基线。常规环境优先按AGENTS/justfile使用just；当前Windows缺sh时，用下面等价展开替代，逐条检查退出码，失败停止放行。不能把启动失败记成测试失败或通过。

## 本轮规划自检

```powershell
uv run --no-sync python .codex-tasks/20261010-unified-characteristics/raw/validate_plan.py --planning
git diff --check
```

--planning验证产品子任务全部仍TODO、9子任务/51叶子步、依赖DAG/核心独立发布、SPEC章节/证据目录、本地链接、设计来源SHA256。实施开始后去掉--planning，并在结构变更时同步校验器的任务/步骤数量。语义覆盖和行为验收仍需独立审查，不由结构脚本代替。
任务目录被git忽略，因此git diff --check不会覆盖这些新文件；结构脚本另查所有新Markdown尾空白/链接，主代理人工核验内容。

## 快速门的PowerShell展开

对应当前just check-fast（含justfile新增的旧模型专项），每条成功后再执行下一条；各包pytest分别调用。

```powershell
uv run --no-sync ruff check .
uv run --no-sync ty check .
uv run --no-sync python packages/suspension_multibody/tests/architecture/legacy_surface_gate.py --check
uv run --no-sync python packages/suspension_kernel/scripts/check_module_layering.py --strict --final
uv run --no-sync python packages/suspension_multibody/scripts/check_composable_release.py --skip-isolation
uv run --no-sync pytest packages/suspension_multibody/tests/architecture/test_no_legacy_vehicle_models.py -q -p no:cacheprovider
uv run --no-sync pytest packages/suspension_multibody/tests --ignore=packages/suspension_multibody/tests/adams --ignore=packages/suspension_multibody/tests/architecture --ignore=packages/suspension_multibody/tests/cases -q -p no:cacheprovider
uv run --no-sync pytest packages/suspension_kernel/tests -q -p no:cacheprovider
uv run --no-sync pytest packages/suspension_contracts/tests -q -p no:cacheprovider
```

本轮另用ruff定向检查规划校验脚本。无修改产品求解代码，不跑下面的数值或慢测试。

## 未来实施的构建与数值门

native/力导数/求解变化需要构建，并在快速门外跑数值门：

```powershell
uv run --no-sync python packages/suspension_kernel/scripts/build_suspension_kernel.py
uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
uv run --no-sync python packages/suspension_multibody/scripts/case_parity_check.py
uv run --no-sync python packages/suspension_multibody/scripts/kc_perf_gate.py
```

等价just build-kernel、just gate-numeric。不能用不带生产者和--actual-dir的kc_parity_check替代数值证据。

## 核心P6/跨系统最终收尾

日常不默认运行；仅P6大重构/跨系统收尾或准备提交运行：

```powershell
uv run --no-sync pytest packages/suspension_multibody/tests/architecture -q -p no:cacheprovider
uv run --no-sync pytest packages/suspension_multibody/tests -q -p no:cacheprovider
git diff --check
```

第二条等价just test-all。新能力仍须各SPEC的专项解析/独立有限扰动/Adams证据和性能预算，旧回归全过不构成新增功能验收。保存实际日志、退出码/误差/缺口；skip/xfail不得增长。
