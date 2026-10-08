# 旧模型迁移到统一多体文档

目标生产入口是 simulate(AssemblyDocument, CaseDocument)。文件路径和 Python 文档对象经过
同一加载、解析、编译、native 提交与 ResultEnvelope 解码。模型不再按悬架或整车类别构建。

## 旧输入离线转换

authoring.migration 中的转换器只生成声明，不调用装配器或求解器：

| 旧数据 | 转换函数 | 输出 |
|---|---|---|
| AxleDeclaration | migrate_v1_axle | 普通 AssemblyDocument |
| AxleDeclaration 与 K/C 输入 | migrate_v1_kc_case | 普通总成及 CaseDocument |
| VehicleDeclaration | migrate_v1_vehicle | 普通总成 |
| VehicleDeclaration 与工况 | migrate_v1_vehicle_case | 普通总成及工况 |
| SI AxleDynamicsModel/Case | migrate_v1_dynamic_axle | 普通总成及工况 |

save_migrated_assembly 将内存声明保存为模板、子系统、属性及总成文件，之后通过同一
DocumentLoader 加载。旧声明类型保留用于离线读取；它们不是生产模拟入口。

## 文档职责

Template 定义拓扑和基本单元；Subsystem 绑定硬点与 Property；Assembly 通过端口组合子系统。
Rig 是普通 Subsystem，提供支撑、作动关节和接口。Case 表达 Study 的采样、激励、输出和边界。

Wheel 是轮体与 Tire 力元的唯一所有者。Brake 与 Drive 是普通子系统，可以声明力元及刚体。
悬架台架与整车使用同一 Wheel/Suspension 模板：台架锁定明确的 spin 坐标，
整车动态释放该坐标；轮胎接触 frame 明确引用非自转 carrier。

Property 保存质量、惯量、本构参数、表格、TIR 数据或公式参数，不承担装配和求解。
测量与结果按稳定实体 ID、frame ID 和 channel 读取，不从对象形状猜测业务类型。

可执行文件及纯内存示例见 composable_extension_examples.md，发布探针实际运行其中四个例子。

## 验证边界

冻结动态输入的 26 份产物保持原哈希。整车数值验收按用户批准的五项严格等价及三项独立
物理差异验收执行，三项原严格不匹配单独保留，不重录旧基线。

本次迁移的删除前退出门必须通过，才会切换公开入口并删除旧生产编译、准备和解码分支。
进度和实际验证记录以 .codex-tasks/20261006-unified-generic-subsystems 的 CSV 与 raw 证据为准。
