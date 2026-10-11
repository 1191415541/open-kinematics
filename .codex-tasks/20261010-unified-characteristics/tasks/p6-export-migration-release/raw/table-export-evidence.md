# P6 表导出步骤证据（2026-10-11）

## 实施

`suspension_multibody/io/artifacts.py` 现在真正执行 `formats=("csv",)`、`formats=("parquet",)` 或两者组合：

- 保留既有 `manifest.json`、`native_result.json`、`arrays.npz`；
- 每个已存通道生成同名 CSV/Parquet 表；
- 表行保留 `sample_index`、`case_index`、`case_name`、`output_sample_id`、`time_s`、`sample_role`、组件索引、原始值、已知 `unit`/`frame_id` 和非有限值 `reason`；
- manifest 的 `tables`、`table_formats`、`table_layouts` 描述实际生成的文件，既有 `layouts` 结构保持兼容；
- 未知格式在创建输出目录前拒绝；测试证明目标目录及父目录均不会被创建。

## 行为验证

```text
uv run --no-sync pytest packages/suspension_multibody/tests/io/test_artifacts_unified.py packages/suspension_multibody/tests/characteristics/test_query_artifact.py -q -p no:cacheprovider
35 passed
```

覆盖：CSV/Parquet 实际文件、格式选择、同值往返、样本身份/时间、单位/frame/reason 列、NPZ 原值、冷读已有路径、未知格式拒绝。该步骤没有声称 P6 整体完成：P4a/P5、R1–R9 消费者迁移和完整发布门仍未满足。
