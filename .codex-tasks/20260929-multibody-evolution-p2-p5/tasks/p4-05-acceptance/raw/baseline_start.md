# p4-05 第 1 步：起点值实测（本行自己跑，不抄子任务结论）

```
$ uv run --no-sync pytest packages/suspension_multibody/tests --ignore=.../adams --ignore=.../architecture --ignore=.../cases -q -p no:cacheprovider
1188 passed, 1 xfailed in 41.10s
exit=0
```

| 计数 | 起点实测值 |
|---|---|
| passed（快速集） | 1188 |
| skipped | 0 |
| xfailed | **1** |
| failed | 0 |

`tests/adams` 的环境 skip 是既有项，本行未跑该目录（不涉及轮胎力律改动）。

**不相关的既有失败：无。** 起点全绿。
