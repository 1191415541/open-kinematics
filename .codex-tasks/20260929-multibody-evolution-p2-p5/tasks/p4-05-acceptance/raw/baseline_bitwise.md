# p4-05 第 4 步：`kc_baseline` 与 `dynamic_hash_baseline` 逐位未变

**不以「相关测试通过」替代逐字节比对。**

## 1. `dynamic_hash_sentinel.py --check`

```
$ uv run --no-sync python packages/suspension_multibody/scripts/dynamic_hash_sentinel.py --check
combined sha256  : fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9
OK: dynamic output matches the frozen baseline byte-for-byte
exit=0
```

`combined sha256` **等于冻结值**（与 `tasks/p1`/p2-01 冻结时记的同一串）。

## 2. `tests/data/kc_baseline/` 三个文件的逐字节判定

本行**自己算的哈希**（不比「测试通过」）：

| 文件 | 字节数 | sha256 |
|---|---|---|
| `k_states.json` | 5058 | `24b138be866f91e7b230cf863472f98bb991ea094edad74e18ed512f015c68a8` |
| `c_states.json` | 94375 | `9b80a05afff4fe37102ac46a075aeadb1d0c45240a93d87c98b0ab4b7fb9c010` |
| `manifest.json` | 417 | `cd4eb1a447e3735b7352916f3c5f279fc0ce0de6b3aa1e399675c292cbb309cf` |

## 3. 工作树层面：这三个文件**一个字节都没动**

```
$ git status --short -- packages/suspension_multibody/tests/data/
（空）
```

`git status` 为空表示该目录下**没有任何**未跟踪/已修改文件——即 `kc_baseline/`
与其它冻结基线在本 Epic 全部已落地行中都**未被重录**。这是比哈希更强的一层：
哈希证明内容一致，`git status` 证明**没人碰过**。

## 4. F17 的说明

`tasks/p4-01-freeze/raw/arb_baseline.md` 记的 F17 事实是「ARB 冻结产物记录的是无 ARB」。
**本行没有据此放松任何判定**：上面的逐字节比对是对整个 `kc_baseline/` 与 26 个
dynamic artifact 做的，不是只看与 ARB 相关的那一项。
