# p4-05 第 6 步：skip / xfail 增量零

**与第 1 步实测的起点值逐项比对**（起点见 `baseline_start.md`）：

| 计数 | 起点（第 1 步实测） | 收尾（本步实测） | 增量 |
|---|---|---|---|
| passed（快速集） | 1188 | 1188 | 0 |
| skipped | 0 | 0 | **0** |
| xfailed | **1** | **1** | **0** |
| failed | 0 | 0 | 0 |

```
$ uv run --no-sync pytest packages/suspension_multibody/tests --ignore=.../adams --ignore=.../architecture --ignore=.../cases -q -p no:cacheprovider
1188 passed, 1 xfailed in 41.10s
exit=0

$ uv run --no-sync pytest packages/suspension_multibody/tests/architecture -q -p no:cacheprovider
147 passed in 633.84s (0:10:33)
exit=0
```

`tests/architecture` 的 skipped 与 xfailed 都是 **0**，与起点一致。

`tests/adams` 的环境 skip：本行未跑该目录（本行与已落地的 p4-02/p4-03/p4-04
都不触及轮胎力律），**未增长**——没有新增任何 skip 语句。

**不相关的既有失败：无。**

**未使用** `-k` / `--deselect` 豁免任何用例。
