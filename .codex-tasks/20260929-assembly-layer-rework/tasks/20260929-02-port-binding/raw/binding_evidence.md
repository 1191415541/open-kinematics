# 配对落成实体的证据（02 第 3 行的判据）

- 执行时间：2026-09-29
- 证据脚本：会话 scratch `binding_probe.py`（未入库；下面是它的**原始输出**）
- 被测代码：`connections/links.py`（`LinkSpec` / `build_links`）+ `subsystems/composition.py`（在 `match_requirements` 之后调用它）

## 1. 显式配对与推断配对的产物逐项一致

同一个装配（一侧提供端口 `mount`，另一侧声明需求 `mount` + 配方）在「配对留空让 matcher 推断」与「文件/调用方显式写出配对」两种输入下的原始输出：

```text
inferred:
  binding      {'mount': 'axle/mount'}
  row          mount_subframe type=WeldJoint kind=weld
  ends         upright_L.mount <-> subframe.mount
  coordinates  a=(0.1, -0.7, 0.3) b=(0.0, -0.4, 0.3)
  port         axle/mount
  connection   Connection(name='mount_subframe', kind='ideal', body_a='upright_L', body_b='subframe', point_a='mount', point_b='mount')
  fingerprint  e5ca19b8cae7089c
explicit pairing:
  binding      {'mount': 'axle/mount'}
  row          mount_subframe type=WeldJoint kind=weld
  ends         upright_L.mount <-> subframe.mount
  coordinates  a=(0.1, -0.7, 0.3) b=(0.0, -0.4, 0.3)
  port         axle/mount
  connection   Connection(name='mount_subframe', kind='ideal', body_a='upright_L', body_b='subframe', point_a='mount', point_b='mount')
  fingerprint  e5ca19b8cae7089c
bodies           ['subframe', 'upright_L']
joints           ['mount_subframe']
points           [('subframe', 'centre'), ('upright_L', 'mount')]
identical        True
no recipe -> rows () joints {}
```

判据逐项：
- **两端体与两端点**：`upright_L.mount <-> subframe.mount`，两端坐标分别是配方给的 `(0.1,-0.7,0.3)`（需求方局部）与端口声明的 `(0.0,-0.4,0.3)`（端口所有者局部）。
- **类型**：`WeldJoint`（理想副，同时进 `joints` 与 `ideal_constraints` 两列），旁边一条 `Connection(kind='ideal')` 行。
- **一致**：两种输入的 bindings、生成的 `LinkRow`、整条装配的指纹（`fingerprint_assembly`，覆盖体/点/关节/力元/连接的集合与点坐标）完全相同 → 显式配对是**覆盖手段**，不是第二条装配路径。
- **零配方即零变化**：`no recipe -> rows () joints {}` —— 贡献里没有配方时生成行为空，既有装配体一个实体都不多不少（这是 01 快照 `--check` 零差异的实现保证）。
- **配对端不依赖名字**：端口挂在 `subframe` 上，它不在任何硬编码集合里，配对仍成立。

## 2. 失败路径点名（不是静默）

`tests/connections/test_links.py` 里逐条断言（本行实跑 `241 passed`）：

| 情形 | 行为 |
|---|---|
| 需求有多个候选端口、文件又没写配对 | `AmbiguousBindingError`，消息列出全部候选端口名 |
| 配对指向本装配没有的端口 | `BindingError`，消息给出配对名、需求 role 与**实际提供的端口列表** |
| 配方引用的 role 从未被绑定 | `BindingError`，消息给出该 role 与已绑定的 role 列表 |
| 配方绑到的端口没有几何框架 | `BindingError`，点名该端口与它的类型 |

## 3. 三类配方都只用仓库既有类型

- `weld` → `WeldJoint`；`revolute` → `RevoluteJoint`（轴由配方按两端各自局部系给出）；`bushing` → `BushingElement`（刚度缺省为零矩阵＝「声明了但不带柔度」，与模板 mount 槽的读法一致）。
- **没有新增任何运动副类型**：新副类型属内核范围（父 Epic 的 Non-Goal）。
