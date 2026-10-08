# 关键交付接口与删除门

## 1. 库存与源码修改顺序

任务 1 步骤 1 先创建只读 `scripts/check_unified_model_migration.py`，该脚本是证据采集工具，不改建模源码。脚本创建前后记录 Git 状态，采集成功之前不得修改产品源码。

输入为当前 checkout 的源码/脚本/CLI/公开导出、实际 preparation/compiler/decoder/rig 注册表、当前文件与内存示例、冻结基线及 Git 状态。输出 `raw/inventory.json`，并保存：

- HEAD/branch、完整 staged 和 unstaged diff、Git status、已有未跟踪内容副本及哈希；不清理他人的改动。
- 每个实际注册键及消费者的 file:line、入口、输入类型、构建链、submit、decoder、现有对应测试和期望迁移目标。
- 普通/显式 axle、vehicle、多轴与可选分支、全部 bench、Adams importer/strict_k/strict_c/reference、CLI、报告与文档例子的映射。
- 四份冻结基线文件指纹、当前可追溯编译模型/输出证据及物理差异清单；原有 FAILED/BLOCKED 保持标识。

采集或验证失败须退出非零并保存 raw 错误报告、已采集部分和未覆盖项，不把不完整 inventory 当成完成。库存必须注明 producer、schema_version、时间和代码指纹。

库存机器字段至少包括 `captured_at`、`production_fingerprint`、`snapshot_complete`、`registrations`、`consumers`、`frozen_fingerprints`。validator 在任务1步骤2及后续任务进入实施状态时检查成功证据和完整库存，防止记录状态跳过采集。

`--check --inventory ...` 是任务 9 步骤 2 的切换前门：读取原库存和新消费者映射，证明全部列项已有新文档、新 resolver/IR 路径及测量通道；核对无遗漏和离线迁移器无 solver/submit 导入。该门实际调用 `planning/checks.py numeric`、`structural` 和对应单路径/spin/parity 测试，保存退出码；任一失败禁止步骤 3。生产切换前只用显式编译/测试输入评价新路径，不对用户开放新旧双生产选择。

## 2. 模型、坐标和接触 Frame

`ResolvedModel` 是模型唯一真源；SolvePlan 只保存边界/激励/研究/输出，不复制模型。Frame 明确 body-local SE3，世界 frame 是合法 ground 引用。用户模板里的 `tires` 可以作为 Wheel Subsystem 内部的复合声明，不需为了命名统一删除这一数据字段；禁止的是总成平级装配独立 tire-law 和按 wheel 角色选择另一条解释器。

Spin Port 指向稳定 `source_joint_id`、两端 frame、轴正方向、单位及参考相位。展开后 resolver 按稳定 ID 和明确连接占用规则验证唯一 joint；缺 joint、双份轴承、不同端口重复制约束须有报错用例。不能仅凭两个 body 之间有任意 revolute 就猜测哪个坐标。

Contact Frame 显式引用非自转 carrier frame，或由 carrier 与安装 frame 派生。它可随 carrier 转向/外倾，只排除轴承的相对 spin。轮体自身 frame 若不满足这一关系必须拒绝，测试需覆盖多周 spin、非零安装姿态和镜像；两端 owner/joint 关系由声明验证。

## 3. Native 旋转边界契约

任务 5 的 schema 与 reader 同时交付：joint coordinate 引用、motion 类型、reference angle、时间函数/程序 ID、初始相位与速度一致性，以及稳定 constraint ID。具体字段落在现有 native Model，不另加 Python runtime。未知 motion/版本或不支持的 prescribed-speed 在 compile 前拒绝。

实现位置残差、速度一致性和 Scalar/Dual 方向导数；locked 只生成一个标量约束。多周相位若需要状态，仅可在明确 trial/accepted 状态布局保存并支持 rollback，禁止残差求值修改 accepted state。新能力需 capability 公告及单一版本/ABI 真源同步，不支持的旧 native 明确拒绝。

约束输出通过稳定 constraint ID 发布 multiplier 与两端物理反力矩、参考 frame 和功率；明确 lambda 到 wrench 的符号/单位映射，测试 locked+torque、prescribed+load、镜像及不同初始化。仅宣布 Jacobian/反力支持不足以 DONE，需实际 Scalar/Dual/FD 与力矩平衡证据。

修改native reader/约束/状态/输出后先执行现有 `uv run --no-sync python packages/suspension_kernel/scripts/build_suspension_kernel.py`，核验两包镜像 metadata 和 ABI/capability 一致，再运行真实二进制提交测试，不能只通过独立C++探针。步骤5-2的新增测试须同时覆盖部署后的reader和约束求解，任务7-4核验全部native提交。

## 4. 物理等价和发布边界

路径迁移的等价场景仍跑冻结数值/性能门，现有数值脚本的模型生产者必须迁到新文档和唯一 resolver，不保留旧运行算法作为数值门后门。原文件哈希不变。

新增 spin 或修正质量账造成的物理差异单独列 producer/input/result/量纲误差和预期影响，不与旧模型要求逐位相同。若旧默认物理无法无损用新图表达，标记发布阻断；只有用户明确同意默认物理更改后才能调整该默认。不能以重录原 baseline 自动过门。

整车接触frame细则见`vehicle_numeric_change.md`，用户于2026-10-07批准。五个等价工况须通过原冻结哈希，且仅三个明确登记工况允许单列`PHYSICAL_DIFFERENCE`和原哈希不匹配；不得跳过整个vehicle_dynamic族。当前物理门须检查允许字段、完整输入、逐通道单位及独立物理界限，其余差异或缺证据仍退出非零。历史随spin参照图不能作为合法新图，也不能通过删除Spin Port或修改resolver后物理IR绕过校验。
