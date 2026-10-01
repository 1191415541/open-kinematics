# p4-03 判据 (c)：`_BODY_ALIASES` 的收口

## 处置：**整表删除**

`subsystems/geometry.py` 的表（改造前）：

```python
_BODY_ALIASES: dict[str, str] = {
    "uca": "upper_arm",     "upper": "upper_arm",
    "lca": "lower_arm",     "lower": "lower_arm",
    "wheel": "upright",     "knuckle": "upright",   "spindle": "upright",
    "tie": "tie_rod",       "tierod": "tie_rod",
}
```

`resolve_body` 的两处使用（`base = _BODY_ALIASES.get(normalized, normalized)` 与
`candidate = f"{_BODY_ALIASES.get(stem, stem)}_{side}"`）改为直接用传入的名字。

## 理由：它就是「按名字猜身份」

这十个别名做的正是**把模型的词汇翻译成本库的体名**——`wheel` / `knuckle` / `spindle`
→ `upright`，`uca` → `upper_arm`。而 `EPIC.md:233` 要求跨边界身份必须来自形式化声明。
本行造出语义化端口之后，这些身份**已经有声明**了：

- 轮端是哪个体 → 悬架模板 `wheel_center` 连接的 `far_owner`（阶段四 p4-02 已改）；
- 防倾杆吊杆插到哪个件 → 模板的 `arb_mount_<side>` 端口的 `owner`（本行）。

别名表要回答的问题，声明已经回答了，而它剩下的能力只有「猜」。

## 实测：删掉之后无任何消费者

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/subsystems     packages/suspension_multibody/tests/connections     packages/suspension_multibody/tests/authoring     packages/suspension_multibody/tests/rigs     packages/suspension_multibody/tests/templates     packages/suspension_multibody/tests/modeling -q -p no:cacheprovider
494 passed in 15.08s
exit=0
```

**删除前没有任何测试依赖它**，删除后一条都没失败——这就是「它其实已经没人用」的证据，
不是「它没用了所以可以删」。

## 保留的解析能力（未删）

`resolve_body` 仍然解析三类**明确**的写法，它们不是名字猜测：

1. 名字**就是**本届装配的体名（精确命中，直接返回）；
2. 名字带侧别后缀（`_L`/`_R`）→ 加侧别后命中；
3. `chassis` → `ground`（单轴装配没有车身，挂在 ground 上；这是**记录在案的约定**，
   写在函数的 docstring 里，且有测试覆盖）。

## 顺带的边界

`geometry.py:102 side_hardpoints` 本身**未动**——它有自己的使用者
（`adams/strict_c.py`、`si_assembly.py`、`authoring/solver.py`），不属本行。
