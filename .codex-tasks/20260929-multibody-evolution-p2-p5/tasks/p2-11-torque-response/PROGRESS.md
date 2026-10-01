# PROGRESS：p2-11 rotational_torque 非零响应专用夹具验收

- 父 Epic：`20260929-multibody-evolution-p2-p5`
- 形态：`single-full`
- 状态：**DONE**
- 来源：`code-reviewer` 裁决新增（delegation `8e86187b`）

## 恢复块

1. `任务:` 用专用两体文档装置独立验收 p2-09 未达成的「非零力矩响应」
2. `形态:` single-full
3. `进度:` 3/3 步骤 DONE
4. `当前:` 已关闭。code-10 首样本范数 1.0 N·m（= max_torque）、逐样本严格等大反向、对照组 0 行、末样本归零
5. `文件:` `packages/suspension_multibody/tests/cases/test_rotational_torque_document.py`（+2 处）+ 本目录 `raw/non_zero_response.md`
6. `下一步:` 无。p2-05 与 p5-04 的 depends_on 已加上本行

## 交付

见 `raw/non_zero_response.md`：装置说明、四条判据的实测读数、整车夹具取不到非零读数的逐条实测、门禁。

**核心结果**（实跑）：
- 三份测试文件 **30 passed**；`dynamic_hash_sentinel --check` 逐字节一致（`fdfd5a6b…eebc9`）
- `code10 rows: 42`（21 样本 × 2 端）；样本 0 两端 `[-0,-1,-0]` 与 `[0,1,0]`（范数 1.0 N·m）；末样本 0.0

## 边界

未改任何生产代码与内核；未改 ABI；未重录基线；未新增 skip/xfail。
