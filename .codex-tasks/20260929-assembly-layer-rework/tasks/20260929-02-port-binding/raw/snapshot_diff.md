# 产物零差异的证据（02 第 6 行）

- 执行时间：2026-09-29
- 判据命令：`uv run --no-sync python .codex-tasks/20260929-assembly-layer-rework/tasks/20260929-01-freeze/snapshot.py --check`

## 落地前

```text
OK: the seven combinations assemble exactly what the snapshot froze
exit 0
```

（这是 02 开工前 01 收尾时的实测输出，也复核了 01 交付的快照本身可用。）

## 落地后

```text
OK: the seven combinations assemble exactly what the snapshot froze
exit 0
```

## 结论

02 的改动**没有移动任何既有产物**：7 个组合映射的 5 个装配产物（`axle_K@kc_quasi_static`、`axle_C@kc_quasi_static`、`axle_K@axle_dynamic`、`axle_C@axle_dynamic`、`vehicle`）逐项不变，`raw/approved_deltas.json` 保持空数组（本行没有登记，也没有登记权限——只有 05 能写登记）。

这与实现是同一个事实的两面：`build_links` 在没有 `LinkSpec` 时返回空 fragment（`raw/binding_evidence.md` 的 `no recipe -> rows () joints {}`），而既有的六条装配路径都不声明配方，所以它们生成的实体与从前逐项相同。

## 中间过程（登记）

第一次实施把 `compose_simulation_assembly` 的 `root_kind` 形参**误删**（同时引入 `pairings`），导致 `si_assembly_for_axle` 调用失败、`snapshot.py --check` 退出 1、`tests/subsystems` 收集失败。主代理当场修复（把 `root_kind` 形参加回，与 `pairings` 并存）后：`snapshot.py --check` 回到 `OK`、`tests/subsystems` 113 passed。此处登记的是「曾经坏过、已修好且复验通过」，不是「从未失败」。
