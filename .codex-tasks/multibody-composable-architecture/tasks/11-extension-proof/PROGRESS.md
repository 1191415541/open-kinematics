# 11 新拓扑与新试验台扩展性实证

## Recovery

- 任务：`11 验证全新拓扑与试验台扩展`。形态：single-full。依赖 10（DONE）。
- 状态见 `../../SUBTASKS.csv` 第 12 行；输入规格 `../../TASKS.md` 第 11 节。
- 主验收：`uv run --no-sync pytest packages/suspension_multibody/tests/composable -q`；核心场景禁 skip/xfail。

## 实测现状（进入本任务时）

- 无 `tests/composable/`。夹具只到「注册模板」这一步，从未以**不同连接图**的形式穿过生产入口。
- 编译器仍知道**模板的零件名**：`cases/kc_quasi_static/contract.py` 在每个调用点写死 `f"upright_{side}"`。一个轮承载体叫别的名字的拓扑（如拖曳臂）会抛 `KeyError: ('upright_L', 'wheel_center')`——这正是「只改注册名不算扩展」的实证反面。
- 显式拓扑（`topology="explicit"`）的构建**硬声明全部四个角色**（`capabilities_for(subsystems={"chassis","suspension","steering","wheel"})`），不看实际产出了什么。一个没有 rack 体的显式模型因此声称有 steering，试验台随之发 rack 驱动坐标，运行在内核里以 `k.axis_map.rack names unknown coordinate rack_drive` 失败——报的是症状不是原因。
- 显式拓扑的形状与镜像约定也不同：**不镜像**，模型须自己声明左右两侧（`WHEEL_CENTER__R` 等）。

## 做了什么

### 编译器去掉模板零件名假设

`contract.wheel_centre_body(assembly, side)`：按**声明**定位承载轮心的刚体——先试约定名（`upright`/`knuckle`/`hub_carrier`/`trailing_arm`），再全表搜索「声明了 `wheel_center` 点的体」。两侧各有多于一个候选时**点名拒绝**（同一侧出现两个轮心是歧义，任选其一等于把车轮装到错的零件上）。`_driven_coordinates` 与 markers 都改用它。

效果：出厂双横臂的答案与从前逐字相同，而没见过的命名也能被编译。

### 显式拓扑的角色由产出推导

`_explicit_roles(bodies, constraints, points)`：有 rack 体 → steering；有点叫 `wheel_center` → wheel；有非底盘自由体且存在约束 → suspension；底盘恒在。不再断言一个固定集合。这样显式模型的 capability 是**结果的函数**，试验台据此收缩——与对称路径从请求读角色是同一条规则。

### 合成夹具（D2 授权，明确标 synthetic）

| 文件 | 内容 |
|---|---|
| `tests/data/composable/synthetic_trailing_arm_axle.json` | 左右拖曳臂单轴。连接图与双横臂**结构性不同**：每侧一个臂、由单个底盘转动副支承，无上臂、无球铰、无拉杆、无齿条。2 个约束 vs 13 个 |
| `tests/data/composable/synthetic_load_bench.json` | 物理加载台：固定框 + 自由加载小车 + 移动副 + 位移驱动 + 弹簧 + 阻尼，全部用**已有物理元件** |

两个夹具都在文件内与加载器中标注 `_synthetic: true` 与 `SYNTHETIC FIXTURE` 原文，并各自用 `expected` 段记录**独立推导**的自由度、约束数、几何斜率与预期力。

夹具的几何/自由度说明（不依赖求解）：

- 拖曳臂每侧 1 自由度：单个转动副移除自由体 6 个自由度中的 5 个，余下绕枢轴 Y 轴的转动。
- 枢轴 `(-350, -500, 250)`，轮心 `(0, -700, 300)` → 相对偏移 `(350, -200, 50)` mm。
- 绕 +Y 转 θ：`z' = -dx·sinθ + dz·cosθ`，故 `dz/dθ|₀ = -dx = -350 mm/rad`。
- 10 mm 轮心行程 ⇒ `θ = 10/350 = 0.02857 rad = 1.637°`。
- 加载台弹簧 10 mm：`k·δ = 25000 × 0.01 = 250 N`。

## 关键实测（全部为数值断言，非 status 检查）

| 判据 | 实测 |
|---|---|
| 拖曳臂经 `api.run_case` 求解 | 3 states 全收敛，约束残差 < 1e-5 |
| 轮心高度 vs 独立旋转公式 | travel −10/0/+10 → z 290.0/300.0/310.0；与精确旋转公式在 0.05 mm 内一致 |
| 左右耦合 | 无右控制时两侧行程相等（shorthand 的对称耦合） |
| 加载台力元进入求解 | 组件表含 `bench_spring_L/_R`、`bench_damper_L/_R`，且 wrench 非零 |
| 硬点扰动 | `WHEEL_CENTER.z += 20` → 求解后轮心 z 恰好 +20（1e-4 内） |
| 姿态扰动（枢轴轴向倾斜 5%） | 同一行程下左外倾角改变（>1e-9），两侧仍收敛 |
| 拓扑 vs 双横臂 | bodies 3 vs 10、constraints 2 vs 13、joint kinds `{RevoluteJoint}` vs 多类、无 rack/tie_rod |
| 同名台跑两拓扑 | 同一 bench 名与 family 名分别命名，同一 model document 逐字相等 |
| 无转向声明 | 单轴请求 brake/drive 被点名拒绝（D3 对合成夹具同样生效） |
| K/C 冻结快照 | kc_parity 容差内，无漂移 |
| 8 个 family | case_parity 全 PASS |

## 门禁设计（`tests/composable/`，16 项）

夹具自证 synthetic 且连接图确实不同（按图断言，不按名字）/ 双横臂反向对照 / 拖曳臂经公共入口求解且残差真实 / 轮心高度对独立旋转推导 / 左右耦合 / 加载台实体声明 / 力元真的进入求解（非查文档键）/ 同一台驱动两拓扑（参数化）/ 硬点扰动传导致解 / 姿态扰动改变解 / 核心源文件集合与夹具位置 / 新台只需注册 `RigSpec`（含 `family` 与 assembly 归属）/ 拖曳臂可转 SI 动态模型 / 不同台名下 model document 逐字相同 / 合成夹具同样受 D3 约束。

## 验证记录

| 命令 | 退出码 | 结果 |
|---|---|---|
| `pytest tests/composable -q` | 0 | 主验收 **16 passed**，无 skip/xfail |
| `pytest tests/composable tests/rigs tests/cases tests/simulation tests/studies tests/subsystems -q` | 0 | 331 passed |
| `python scripts/kc_parity_check.py --check` | 0 | 无漂移 |
| `python scripts/case_parity_check.py` | 0 | 8 families 全 PASS |
| `ruff check .` / `ty check .` | 0 | 全通过 |

## 未覆盖与保留

- 合成夹具**不提供转向**，遵守单轴无制动/驱动规则；不声称任何工程精度（D2）。其几何、质量、自由度和预期运动已在夹具 `expected` 段显式记录。
- 动态读数的**实跑**未做：本任务的动态一侧验证的是拖曳臂可被转为 SI 动态模型（`axle_dynamics_model`）且体/关节/质量一致。要实跑需带时程与轮胎力律的动态 case，属另一个夹具。
- 「核心源文件哈希不变」以**核心文件集**的形式断言（夹具只落在 `tests/` 下），未钉字面摘要——钉字面值会在任何合法的别处改动上误报。真实的本任务核心改动（去模板名假设、角色推导）本身就属 TASKS 写范围允许的「必要显式注册」之外的修正，已在本次交付内完成并留下测试。
