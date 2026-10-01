# p3-04 证据 (c)：4 轮与改造前逐位一致（零回归硬门）

判据对应 `EPIC.md` 行 255 的 **(c)** 与 `SUBTASKS.csv` 的 `p3-04` `notes`：
「4 轮逐位一致是本行硬门」。生产消费点是 `vehicle/service.py:40`。

## 0. 比较什么、怎么比

**改造前的实现不是记录下来的数字，而是文件本身**：改造前 `git show HEAD:` 的内容
原样存档为 `raw/static_loads_before.py`。两者同源，实测如下：

```bash
git show HEAD:packages/suspension_multibody/src/suspension_multibody/vehicle/static_loads.py | md5sum
md5sum .codex-tasks/.../p3-04-static-loads/raw/static_loads_before.py
diff <(git show HEAD:.../vehicle/static_loads.py) .codex-tasks/.../raw/static_loads_before.py
```

```
e8ee0b870a9ebde39034fb7b9582081e  -
e8ee0b870a9ebde39034fb7b9582081e  .../static_loads_before.py
（diff 无输出）ARCHIVE_IDENTICAL_TO_HEAD_LF
EXIT=0
```

md5 `e8ee0b870a9ebde39034fb7b9582081e` 是 LF 规范化后的内容哈希；工作区检出态因 CRLF 为
`2b989cfe078bacc5a5969dbb4ba19250`，字节内容相同。

比较程序：`raw/four_wheel_bitwise_probe.py`。它在**同一个进程里**把存档文件当作独立模块
载入（`importlib`），与改造后的模块在**同一份输入**上各跑一次，逐字段比较
**IEEE-754 双精度的原始字节**（`struct.pack(">d", value).hex()`），不是 `isclose`：

```bash
uv run --no-sync python .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-04-static-loads/raw/four_wheel_bitwise_probe.py
```

```
EXIT=0
DIFFERENCES: none
```

完整输出（每个字段的 before/after 十六进制字节）见 `raw/four_wheel_bitwise_output.txt`。

## 1. 比较的字段

每个工况都比较这些字段的原始 8 字节：

- `rank`、`unique`（改造后由 `rank == N` 得出，与改造前 `rank == 4` 的语义对照）
- `residual`、`total_mass`、`center_of_mass`（三分量逐分量）
- `wheel_loads` 的 **4 个键各自的值**
- `support_points` 的 **4 个键各自的点**（三分量逐分量）
- `summary.total`（走 `WheelLoadSummary`，即生产报表链）
- 键集合是否相等

## 2. 6 个工况（覆盖 `acceleration` 与 `gravity` 两条输入轴）

| # | 工况 | `acceleration` | `gravity` |
|---|---|---|---|
| 1 | 静态 | `None` | 9810.0 |
| 2 | 纵向 | `[1000, 0, 0]` | 9810.0 |
| 3 | 侧向 | `[0, 1000, 0]` | 9810.0 |
| 4 | 垂向 | `[0, 0, -500]` | 9810.0 |
| 5 | 纵向+侧向+垂向 | `[-3000, 2000, 100]` | 9810.0 |
| 6 | 月球重力（改重力而非改加速度） | `None` | 1620.0 |

每个工况的每一个字段都是 `OK`，**没有任何 `DIFF`**。

## 3. 逐位一致的实例（工况 1，静态）

```
  OK  rank/bool              before=3 after=3
  OK  unique(rank==N)        before=False after=False
  OK  residual               before=3eb0000000000000 after=3eb0000000000000
  OK  total_mass             before=409701a53bf7ca49 after=409701a53bf7ca49
  OK  center_of_mass         before=0000000000000000000000000000000040328e91cd8e44da
                             after =0000000000000000000000000000000040328e91cd8e44da
  OK  load front_left        before=414b8ce86e9feb2e after=414b8ce86e9feb2e
  OK  load front_right       before=414b8ce86e9feb2d after=414b8ce86e9feb2d
  OK  load rear_left         before=414b8ce86e9feb2d after=414b8ce86e9feb2d
  OK  load rear_right        before=414b8ce86e9feb2d after=414b8ce86e9feb2d
  OK  support front_left     before=4095e00000000000c0877000000000000000000000000000
  OK  support front_right    before=4095e0000000000040877000000000000000000000000000
  OK  support rear_left      before=c095e00000000000c0877000000000000000000000000000
  OK  support rear_right     before=c095e0000000000040877000000000000000000000000000
  OK  summary.total          before=416b8ce86e9feb2e after=416b8ce86e9feb2e
  OK  wheel name set         before=['front_left','front_right','rear_left','rear_right']
                             after =['front_left','front_right','rear_left','rear_right']
  np.array_equal on wheel-load vectors: True
```

即 `3611088.8642553305 / 3611088.86425533 × 3`（十六进制
`0x1.b8ce86e9feb2ep+21` / `0x1.b8ce86e9feb2dp+21`），与改造前逐位相同。

## 4. 生产消费点 `vehicle/service.py:40` 的路径

`compute_static_wheel_loads(model).wheel_loads`（`service.py:40` 取的正是这个字段）：

```
front_left  0x414b8ce86e9feb2e
front_right 0x414b8ce86e9feb2d
rear_left   0x414b8ce86e9feb2d
rear_right  0x414b8ce86e9feb2d
```

未改 `vehicle/service.py`：改造后的报错类型仍是 `ValueError` 子类，
`:41` 的 `except (ValueError, np.linalg.LinAlgError)` 行为不变。
`packages/suspension_multibody/tests/vehicle/test_service_contract.py` 全绿（见 `raw/run_log.md`）。

## 5. 零回归的其它证据（同一批实跑）

| 命令 | 退出码 | 结果 |
|---|---|---|
| `uv run --no-sync python .../scripts/dynamic_hash_sentinel.py --check` | 0 | `OK: dynamic output matches the frozen baseline byte-for-byte`；`combined sha256: fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9` |
| 同上，把 `static_loads.py` 临时恢复成 HEAD 版本再跑一次（对照组） | 0 | **同一** `combined sha256`、同一行 `OK` |

对照组的含义：该 sentinel 的结论与 `static_loads.py` 是否改造**无关**
（它比对的是 K/C 与动力学输出哈希，不含静态轮荷），所以这条是「没有重录任何基线」的
旁证，而不是本行改造导致的差异。`git status` 中 `tests/data/**` 与
`dynamic_hash_baseline.json` 均未被改写。

## 6. 为什么 N=4 一定逐位相同（独立于结果字节的理由）

`EPIC.md` 行 228 要求这类说明独立于结果字节。四条：

1. **自由度与约束行数不变**：N=4、3 条平衡方程；矩阵仍是
   `[ones(4); x_i − com_x; y_i − com_y]`，右端项仍是
   `[m(g+a_z); −m·h·a_x; −m·h·a_y]`。改造只把「列的来源是 `_WHEELS` 常量」
   换成「列的来源是装配轮端表的插入顺序」，而两轴 `VehicleModel` 的装配顺序实测就是
   `front_left, front_right, rear_left, rear_right`（与旧常量逐字相同）。
2. **力路径不变**：仍是 `np.linalg.lstsq(matrix, rhs, rcond=1e-12)` 的最小范数解，
   同样的输入数组、同一次库调用；新增的 `residual_tolerance` / `unique` 只是**读**，
   不参与求解。
3. **接触点不变**：`_support_points` 从「遍历 `vehicle.wheels` 再查
   `assembly.wheel_centers[wheel.name]`」改为「直接遍历 `assembly.wheel_centers`」。
   两者对两轴车给出同一组 `(body, local)` 与同一顺序 —— `wheel_centers` 本来就是按
   `vehicle.wheels` 的顺序构建的（`subsystems/assembler.py:427-465`），
   因此每个 `support_points` 值仍来自同一次 `point_world` 调用。
4. **新的报错路径不触发**：N=4 且几何正常时 `residual` 远小于容差
   （实测 9.54e-07 << 1.44e-02），残差判据不会介入；旧的 `rank < 3` 判据在正常 4 轮上
   本就不触发（实测 `rank = 3`）。两条判据在四轮上都不改变结果。
