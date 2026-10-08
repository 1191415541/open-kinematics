# 通用子系统单一路径重构 Epic

## Goal

把悬架、整车、试验台、车轮、轮胎、制动和驱动全部迁移到同一套 Template -> Subsystem -> Assembly -> Study -> Native Solver 路径。生产代码只保留一个解析后模型、一个编译入口、一个提交入口和一个结果信封；轮胎属于 Wheel Subsystem 内部的 Tire Element，Brake 和 Drive 是与 Suspension、Steering 同级的 Subsystem。

车轮自转由同一个 Wheel Subsystem 的显式 Spin Port 和 Study/Case 边界条件决定：悬架 K/C 台架锁定相对旋转自由度，整车动态释放该自由度，不能通过两份悬架定义解决。

## Non-Goals

- 不在本 Epic 内实现 FTire/CDTire DLL、柔性轮辋、完整 Adams 控制器状态方程或新的 GUI。
- 不以删除断言、重录冻结基线或增加 skip/xfail 解决迁移问题。
- 不保留旧兼容路径作为生产仿真后门；旧文件只允许由离线迁移工具转换为新文档。
- 不承诺第一阶段的轮胎或动力总成物理覆盖超过 Adams/Car；本 Epic 的目标是统一建模语义和执行路径。

## Constraints

- Python 侧只接受声明数据和不可变运行输入；native 侧执行版本化契约和状态生命周期。
- 文件对象和 Python 对象必须经过同一解析、解析后模型、编译和结果解码流程。
- 不增加第三方依赖；遵循仓库 `AGENTS.md` 的 ruff、ty、架构门、快速门和数值门。
- 生产编译器不得按 `axle`、`vehicle`、`suspension`、`wheel`、`brake`、`drive` 分支创建实体。
- Wheel Subsystem 是车轮刚体和 Tire Element 的唯一声明者；Rig 不得创建第二个车轮或轮胎。
- Brake 和 Drive Subsystem 可以只有力元，也可以声明制动盘、卡钳、轴系和传动刚体；它们不降级为全局 LawInstance。
- 旋转约束只能由明确的 Joint、Spin Port 和 Boundary Condition 产生，禁止隐式锁定所有轮体姿态。

## Risk Assessment

- 生产路径迁移会触及 `si_assembly`、车辆总装、试验台接线、Study、native emitter 和结果解码，必须先保存解析后模型和数值指纹。
- 车轮自转边界错误会造成过约束、漂移或接触 frame 随轮自转，需同时验证悬架台架和整车动态。
- 旧 family 结果消费者可能依赖对象形状；结果统一必须通过稳定实体 ID 和通道 schema 迁移，不能靠适配器继续保留第二条求解路径。
- 删除兼容模块是高风险变更，必须在所有生产调用迁移后执行，并由 AST 门阻止旧入口复活。
- 库存、消费者、工作区快照及冻结指纹必须在任何产品源码修改之前采集；库存或删除前退出门失败时禁止切换和删除。

## Child Deliverables

1. 统一领域术语、Schema 和解析后模型 IR。
2. 文件/Python 文档加载、资源清单和同一解析入口。
3. 平等子系统模型：Wheel/Tire、Brake、Drive 与 Suspension/Steering。
4. 类型化端口、Communicator 和普通 Rig 连接。
5. Wheel Spin 端口及悬架台架/整车统一边界条件。
6. 统一本构力元生命周期和 native law contract。
7. 统一 Compiler、Study 和 Backend Emitter。
8. 统一 ResultEnvelope、测量和结果查询。
9. 全部生产路由迁移并删除兼容执行路径。
10. 数值、性能、架构、文档和最终退役验收。

## Dependency Notes

- 2、3 依赖 1；4 依赖 1 和 3；5 依赖 3 和 4。
- 6 依赖 1 和 3；7 依赖 1、2、4、5、6；8 依赖 7。
- 9 依赖 2、3、4、5、6、7、8，并在删除前完成旧路径与新 IR 的 parity。
- 10 依赖 9，负责删除后的架构门、冻结数值与交付验证。
- `depends_on` 使用 `;` 分隔多个依赖。

## Done-When

- [x] `SUBTASKS.csv` 全部 10 行 DONE。
- [x] 生产代码只有一条 Model IR -> Compiler -> Native Backend -> ResultEnvelope 路径。
- [x] Wheel（内部含 Tire）、Brake、Drive 都是普通 Subsystem；Rig 不创建重复车轮。
- [x] 悬架台架和整车使用同一 Wheel/Suspension 定义，只改变 Spin Boundary 和其他端口连接。
- [x] 旧兼容执行路径和公开导出删除，旧文件仅能通过离线迁移器转换。
- [x] 快速门、求解相关数值门、架构退役门、文件/Python parity 和最终相关测试通过，skip/xfail 不增长。

用户于2026-10-07批准整车接触frame数值验收修订：五个等价工况仍须通过原严格哈希；三个已登记工况须通过完整输入及通道证据和独立物理门，原严格不匹配单列。机器门和全部消费者退出门通过前仍禁止切换和删除。不得把八项原严格门表述为全部PASS，具体边界见`planning/interfaces.md`和`planning/vehicle_numeric_change.md`。
