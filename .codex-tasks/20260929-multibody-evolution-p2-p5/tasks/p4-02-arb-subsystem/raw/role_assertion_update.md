# p4-02 判据 (e)：角色断言更新与理由

## 改动

`packages/suspension_multibody/tests/templates/test_template_model.py`

| | 改前 | 改后 |
|---|---|---|
| 测试名 | `test_six_roles_are_declared` | `test_seven_roles_are_declared` |
| 期望值 | `("brake","chassis","drive","steering","suspension","wheel")` | 同一元组**加入** `"anti_roll_bar"`（排序后它排在最前） |

```python
def test_seven_roles_are_declared() -> None:
    # `anti_roll_bar` joined the table in stage four: the bar spans both sides of
    # one axle, so it cannot be owned by either side's suspension template.
    assert role_names() == (
        "anti_roll_bar", "brake", "chassis", "drive",
        "steering", "suspension", "wheel",
    )
```

## 理由（不是「测试挂了所以改」）

1. **该测试断言的就是角色表的成员集合**，而本行的交付正是往角色表加一个角色。
   集合变了而断言不变，两者就有一处在说谎——测试是期望值的持有者，所以改它。
2. **改名而不是只改元组**：名字里的 `six` 是断言内容的一部分（它声明了基数），
   留着旧名会让下一个读者以为这张表还是六个。
3. **没有放宽**：元组是**全等比较**（`==`，不是子集或 `in`），
   新增角色以外任何增减都会失败。基数从 6 变 7 是本次的**预期**变化，已在上面写明。
4. 同一文件里的 `test_only_brake_and_drive_carry_a_torque_channel` 与
   `test_every_role_declares_mounts_slots_and_outputs` **未改**，因为新角色
   `has_torque_channel=False`，且它声明了 mounts/slots/outputs，两条断言继续成立。

## 实测

```
$ uv run --no-sync pytest packages/suspension_multibody/tests/templates packages/suspension_multibody/tests/subsystems packages/suspension_multibody/tests/modeling -q -p no:cacheprovider
298 passed in 5.83s
exit=0
```
