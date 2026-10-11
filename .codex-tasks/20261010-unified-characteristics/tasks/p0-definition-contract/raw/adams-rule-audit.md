# Adams 规则审计说明

本文件是 `raw/adams-rule-audit.csv` 的叙述性说明。CSV 是机读台账，本文件记录不可压缩为字段的环境事实与推理链。

## 对标版本与本机环境

- 对标目标版本：**Adams Car 2024.1**（设计 §11.1 锁定）。
- 审计日期：2026-10-10（Asia/Shanghai）。
- **关键事实：本机没有安装 Adams Car 2024.1。** 核查证据：
  - `C:\Program Files\MSC.Software` → 不存在（`Test-Path` 返回 False）。
  - 全盘检索 `turn.html` / `example_o_wheelenv.html`（C:/E:/D:，递归，300 s 上限内未命中设计所引路径）。
  - 注册表 `HKLM\SOFTWARE\WOW6432Node\MSC.Software Corporation` 下只有 MSC Apex Documentation，无 Adams 项。
  - 本机实际安装：`G:\MSC.Software\Adams\2025_1_1`（`help\adams_car\` 存在）。
  - `C:\adams_work\analytic_adams_car_home\.msca\` 内有 `msca_2024_1.reg` 与 `.acar.cfg`，说明**历史上曾配置过 2024.1 的 ACar 运行环境**，但安装目录与 help 已不在本机；该 `.reg` 仅为注册表导出片段，不含 help 内容。

结论：设计 §11.1 给出的两条引用路径在当前环境**不可复现**。

## 路径映射差异（版本不一致的直接证据）

| 设计 §11.1 声称 | 2025_1_1 实际 |
|---|---|
| `.../help/adams_car/example_o_wheelenv.html` 给出 `C=dX/dF`、12×12 顺序、增量柔度 | 该文件是**车轮包络输出文件（.wev）格式页**，与柔度矩阵无关 |
| `.../help/adams_car/turn.html` 给出 Wheel Rate/Ride/Roll | `turn.html` 是 **Turn Radius** 页；Wheel Rate 实际在 `wheel_1.html`，Ride Rate 在 `ride1.html`，柔度矩阵在 `steer_axis.html` |

两处路径与内容错位，本身即表明设计所引为 2024.1 的页面结构，当前无法据 2025_1_1 直接复现。

## 替代证据指纹（Adams 2025_1_1）

| 文件 | 字节 | SHA256 |
|---|---|---|
| `steer_axis.html` | 15544 | 84579edb58357076220140239f672d132d66afd13818ffa46b9f2c4572ad9409 |
| `susp_chars.html` | 35751 | a415223f72bc513eba3af8ab959a7c870bde7e43cd94cdc47a85c688117362c4 |
| `wheel_1.html` | 10782 | 5f726fa757d3ac29535a4846acd0fffc8fcf2dd5640f813b2f5be410dd420c6a |
| `ride1.html` | 12787 | 0e0b211942cb45d643f14d51095306873c8896f3eae3e11fb2f35a0f9d43c901 |
| `turn.html` | 14569 | 69f5ee1f401d3d41962875f72a089322329d719aefeea42ed9eefdb41dbd168e |
| `acker.html` | 10707 | bd4e0beb6ad65c1c8c7f146e736cfa584e112fff91698b2b9aceca1679c8ec46 |

## 已核实的公式级内容（2025_1_1）

`steer_axis.html`：

- 悬架分析共计算 38 项特性，来源为悬架几何、悬架柔度矩阵，或两者。
- `[C] = [∂X/∂F]`；线性假设下 `X = C F`；`cij` = 自由度 i 的位移 / 自由度 j 的单位力。
- Adams Car 使用 **12×12** 矩阵，关联左右轮心运动与施加在轮心的单位力/力矩。
- `C(3,3)` = 左轮心竖向力引起的左轮心竖向运动；`C(3,9)` = 右轮心竖向力引起的左轮心竖向运动；独立悬架无稳定杆时 `C(3,9)=0`。
- **在悬架 articulation 经过的每一个求解位置计算柔度矩阵。**
- Steer Axis 有两种方法：Geometric Method（过两个不重合硬点连线）与 Instant Axes Method（锁住弹簧行程，施加 3 力 3 矩增量，由轮架平移/转动求瞬轴）。

`wheel_1.html`：`wheel_rate.left = 1/(C(3,3)+C(3,9))`，`wheel_rate.right = 1/(C(9,3)+C(9,9))`。

`ride1.html`：`Ks` = Wheel Rate；串联式 `Ks*Kt/(Ks+Kt)`；测量在轮胎接地点相对车身。

`turn.html`：`turn_radius` 由 steer angle 符号判定内侧轮；输入 Steer angle、Track width、Wheelbase、Wheel-center orientations。

`Dynamic_Analysis.html`：动态悬架分析中该特性计算可用（C++ Solver），但 Jack Excitation Mode 设为运动时结果受限。**未给出完整动态冻结算法。**

## 六组规则结论

六组**全部** `pending_definition`，无一组可判 `verified`：

- G1 阻尼与速度：help 只写 `∂X/∂F`，未展开残差形式，未说明固定哪些速度分量、是否含阻尼位姿/速度切线。
- G2 预载与约束：未提约束/力元预载几何项、乘子增量、接触活动分支。
- G3 惯性：完全未说明加速度/陀螺项参与与否，也未说明冻结何种 wrench。设计 §6.5 的 `frozen_inertial_load_v1` 是**本项目定义**，不得宣称 Adams 等价。
- G4 内部状态：未提轮胎松弛、衬套历史、摩擦/return mapping 的固定或演化协议。
- G5 整车边界：只讲轮心载荷与 12×12 布局，未讲车身 fixed/free、转向保持、其他轮端、轮胎范围、加载反作用路径。
- G6 表示与公式：**部分核实**（见上），但相对参考体、轴正向、角度参数化/单位、track/侧倾角与接地点变换未知；且结论均来自 2025_1_1，2024.1 同页内容未核实。

## 阻断范围

- 两层缺口：(a) 目标版本 2024.1 未安装、引用路径不可复现；(b) 即使按 2025_1_1，动态冻结规则在官方 help 中也未被完整给出。
- 独立参考数据（reference fingerprint）**尚无**：保持 `pending_reference`。
- 按 EPIC §36 与 P0 SPEC §41–42、§84–85：本缺口**不阻断 P0 DONE**（已如实登记为 pending 并明确阻断兼容路径），但**严格阻断 P5 与 P6 的完整 Adams 兼容发布**。
- 后续推进 P5 的前置：取得 2024.1 本地 help（或官方等价物）+ 独立参考数据；或取得用户对变更对标版本/边界的明确授权。
