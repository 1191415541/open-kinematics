# Adams 规则审计说明（P0 修订版，2026-10-13）

本文件是 `raw/adams-rule-audit.csv` 的叙述性说明。CSV 是机读台账，本文件记录不可压缩为字段的环境事实与推理链。

**本版修订的触发条件**：2026-10-13 实测证实**本机安装了 Adams Car 2024.1**，原版（2026-10-10）赖以成立的
前提「本机没有安装 Adams Car 2024.1」**已被证伪**。原版按裁定保留在
`backups/20261013-p0-adams-2024-1-audit/adams-rule-audit.md`，不删除。

## 1. 与原版（2026-10-10）的差异

| 项 | 原版（2026-10-10） | 本版（2026-10-13） |
|---|---|---|
| 环境前提 | 「本机没有安装 2024.1」，改用 `G:\MSC.Software\Adams\2025_1_1` 作替代证据 | **2024.1 已安装**（`C:\Program Files\MSC.Software\Adams\2024_1`），全部证据取自该版本本地 help |
| 证据版本一致性 | 版本列写 `Adams Car 2024.1`，`source_page` 却是 `G:\...\2025_1_1\...`（**自相矛盾**） | 版本列与路径列**一致**，均为 2024.1 |
| G6 | `pending_definition`（「2024.1 未核实」） | **`stated`**（关键公式与布局逐字可引） |
| G3 | `pending_definition`（「动态段不给出完整算法」） | **`partial`**（`adams_solver/gtcmat.html` 明确给出 C++ Solver 动态柔度算法，旧结论对 Solver 手册不成立） |
| G1/G2/G5 | `pending_definition` | `partial`（有关键陈述，但核心项仍缺） |
| G4 | `pending_definition` | `not_stated`（明确「无相关陈述」，比 pending 更精确） |
| 原版的三处「未知」 | 相对参考体、角度参数化/单位、track 与接地点变换 | 在 2024.1 中**均可逐字引用**，已从未知项移除 |

**修订的性质**：原版的问题是**证据版本错位**——它把 2025_1_1 的页面结构当作 2024.1 的，
因而在 2024.1 的页面里找不到内容（原版自述的两处路径错位正是这个后果），并把结论记成
`pending_definition`。2024.1 安装后复核表明：**错位的是证据来源，不是 Adams 的规则内容**。

## 2. 本机环境与对标版本

- 对标目标版本：**Adams Car 2024.1**（设计 §11.1 锁定，未变更）。
- 修订日期：2026-10-13（Asia/Shanghai）。
- 安装根：`C:\Program Files\MSC.Software\Adams\2024_1`（该路径在 `adams/probe.py` 的
  `INSTALL_ROOTS` 内；原版只核对了 `G:\...` 与 `C:\MSC.Software\...`，**漏掉了这一条**）。
- 许可：`MSC Licensing\Lithium`；探测许可通过（`license_probe = passed`）。
- 证据口径：**全部引用 `.../Adams/2024_1/help/` 下的本地官方 help**，不使用 2025_1_1，
  不使用在线文档替代本地证据。

## 3. 与 P5 的关系（真源划分）

- **P0 的六组规则审计是「定义冻结与阻断清单」**：回答「哪些 Adams 规则有出处、哪些未知、
  未知项阻断什么」，是 SPEC §59 的交付物。
- **P5 的 `tasks/p5-adams-svc-parity/raw/adams-rule-audit.csv` 是同一六组的逐项对标真源**
  （2026-10-13 重建，含逐字引文、页面定位表、SVC 页面级差异）。
- 两份 CSV 的**六组 ID 与列语义相同**，本版内容与 P5 权威版一致（同一批本机 2024.1 help 证据）。
  后续 Adams 对标推进以 **P5 版为准**；本文件保留为 P0 范围内可自足的阻断清单。
- 本版**不改变** P0 的既有判定：G1–G6 中只有 G6 达 `stated`，**Adams 兼容发布仍被阻断**
  （见 §5）。

## 4. 逐规则结论（依据 2024.1 help）

- **G1 阻尼与速度：`partial`**。SVC 路径只给位姿偏导定义（`svc_d_5.html`），
  `adams_car` 全手册 `freez|frozen` 0 命中；唯一相关陈述在 Solver 的 `gtcmat.html`
  （速度/加速度扰动项可忽略），但其语义归属 C++ Solver 线性化，不是 SVC 冻结协议。
- **G2 预载与约束：`partial`**。约束与刚度移除有逐项公式（SVCRMV/SVCRMK，`svc_d_5.html`），
  车身约束语义明确；**预载几何项、乘子增量、SVC 路径内的接触分支处理均未陈述**。
- **G3 惯性：`partial`（关键结论已改变）**。`adams_solver/gtcmat.html` 明确给出 C++ Solver
  的动态柔度算法（系统线性化且不改变当前状态 → 输入输出表示 → 速度/加速度扰动项可忽略，
  可在动态或准静态仿真中调用）。因此原版「动态段不给出完整动态冻结算法」的说法
  **对 Solver 手册不成立**；但「冻结何处/何参考系 wrench、陀螺项是否参与」仍**未陈述**，
  故 G3 不能整体判 `stated`。设计 §6.5 的 `frozen_inertial_load_v1` 是**本项目定义**，
  不得宣称 Adams 等价。
- **G4 内部状态：`not_stated`**。轮胎松弛、衬套历史、摩擦/return-mapping 的
  freeze-vs-evolve 协议在 SVC 全部页面 0 命中（`evthelp_svc.html` 只讲局部线性化语义）。
- **G5 整车边界：`partial`**。车身 fixed/free、转向 compliance 要求、轮胎范围
  （SVCRMK 移除 `[Kt]`）均有逐字陈述（`svc_d_2.html`）；**「另一侧轮端」的显式边界与
  载荷反作用路径未陈述**。
- **G6 表示与公式：`stated`**。`[C]=[dX/dF]`、12×12 左右轮心顺序、`C(3,3)/C(3,9)/C(9,3)/C(9,9)`
  索引、`Displacements and forces are calculated in global coordinates`、
  `wheel_rate.left=1/(C(3,3)+C(3,9))`、`wheel_rate.right=1/(C(9,3)+C(9,9))`、
  ride rate 串联式 `Ktotal = Ks Kt / (Ks + Kt)`、roll rate 轮心 vs 接地点（`+ 2/Kt` 之差）、
  角度换算 `(pi/180)`、single bump 对照式，全部逐字可引。

## 5. 阻断范围（本轮更新）

- **软件/许可阻断：已解除**。2024.1 已安装、许可探测通过、
  `validate-adams --smoke --require-installed` 实跑 exit 0。
- **定义审计阻断：部分收窄**。G6 由 `pending_definition` 提升为 `stated`；
  G1/G2/G3/G5 仍为 `partial`，G4 仍为 `not_stated`——**Adams 兼容的完整动态冻结等价声称仍被阻断**。
  任何未知项不得默认使用本项目规则（设计 §618）。
- **独立参考阻断：仍在**。`raw/reference-manifest.json` 保持 `pending_reference`；
  P5 的逐点对标需要与统一特性请求同模型/同边界的独立 Adams 参考，当前尚未生成。
- 按 EPIC §36 与 P0 SPEC §41–42：定义缺口**不阻断 P0 DONE**（未知项已如实登记并明确阻断兼容
  路径），但**严格阻断 P5 与 P6 的完整 Adams 兼容发布**。

## 6. 复核链

- 本文件与 CSV 的每条引文均由主代理**在本机 2024.1 help 上逐字核对**。
- CSV 的 `source_sha256` 列**不是手写的**：由写入脚本
  `backups/20261013-p0-adams-2024-1-audit/write_p0_audit.py` 在写入时对
  `C:\Program Files\MSC.Software\Adams\2024_1\help\...` 的对应文件**实时计算**
  （页面缺失即中止，不会写入占位值）。实测复核：`svc_d_5.html`→`0915d926…`、
  `gtcmat.html`→`e9308542…`、`example_o_wheelenv.html`→`f5f0596d…` 与独立计算一致。
- 页面定位差异（原版 5 个 2025_1_1 文件名在 2024.1 的实际对应）见 P5
  `raw/adams-rule-audit.md` §3。
