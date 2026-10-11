# 统一多体特性分析：Taskmaster 项目计划

- 标识：20261010-unified-characteristics；形态：epic；规划日期：2026-10-10（Asia/Shanghai）。
- 本轮只落盘计划/全部SPEC，不启动源码实施、商业软件或外部数据生成。
- 设计真源：[unified_characteristics_design.md](../../packages/suspension_multibody/docs/unified_characteristics_design.md)；物理定义与数学细节以原设计为准，计划负责范围/步骤/依赖/验收。
- 实施真源：[SUBTASKS.csv](SUBTASKS.csv)；规划真源：[planning/TODO.csv](planning/TODO.csv)；恢复：[PROGRESS.md](PROGRESS.md)。
- 9个single-full子任务、51个叶子步骤；每任务SPEC/TODO/PROGRESS已预建，实施状态全部TODO。
- 独立审查：[REVIEW.md](REVIEW.md)；当前源码落点：[raw/source-evidence.md](raw/source-evidence.md)。现有基础不算新增能力交付。
- 可复现自检与Windows无sh的等价门禁：[VALIDATION.md](VALIDATION.md)。

## 目标与边界

统一悬架/整车×准静态/动态的声明、逐样本响应、指标、查询和导出，沿用唯一ResolvedModel→ResolvedSolvePlan→compile_resolved→run_compiled→ResultEnvelope。特性是输出请求，不造Study、Case family或第二入口。
核心交付当前构型几何/输入偏导、平衡响应、真实时间变化率与原动态工作点frozen_force_tangent/frozen_load_compliance，实际生成Wheel/Ride/Roll/载荷Toe/Camber等数值、身份和诊断，以Adams Car 2024.1独立参考逐项验收。
几何/受载中心、输入偏导/时间变化率、冻结刚度柔度/连续响应分别命名。任意具名载荷/测量模式是核心，12×12只是可选表示；Kzz不是Wheel Rate。

不包括硬点优化、二阶/伴随参数梯度、频响、新力律、新依赖、新仿真入口。仅清理本次迁移的副本/废弃引用，不清无关死代码，不默认删除公共API/核心文件。

## 子任务与依赖

| ID | SPEC与交付 | 直接依赖 | 发布归属 | 叶子步骤 |
|---|---|---|---|---|
| P0 | [SVC 定义、边界与端到端契约](tasks/p0-definition-contract/SPEC.md) | — | 核心 | 6 |
| P1 | [准静态测量、运动学与兼容迁移](tasks/p1-measurement-kinematics/SPEC.md) | P0 | 核心 | 6 |
| P2 | [共享物理切线、平衡响应与指标](tasks/p2-equilibrium-response/SPEC.md) | P1 | 核心 | 6 |
| P3 | [动态接受态调度、几何值与时间变化率](tasks/p3-dynamic-sampling/SPEC.md) | P1;P2 | 核心 | 6 |
| P4a | [原动态样本冻结态刚度、柔度与派生指标](tasks/p4a-frozen-local-response/SPEC.md) | P2;P3 | 核心 | 6 |
| P4b | [连续动态描述符与状态/载荷扰动](tasks/p4b-continuous-descriptor/SPEC.md) | P0;P3 | 独立 | 5 |
| P5 | [Adams Car 2024.1 SVC 逐项独立对标](tasks/p5-adams-svc-parity/SPEC.md) | P0;P1;P2;P3;P4a | 核心 | 5 |
| P6 | [查询/实际表导出、退役收口与核心发布](tasks/p6-export-migration-release/SPEC.md) | P1;P2;P3;P4a;P5 | 核心 | 6 |
| P7 | [大型模型无矩阵与预条件后端扩展](tasks/p7-large-model-backend/SPEC.md) | P2;P4a | 独立 | 5 |

- 核心链：P0→P1→P2→P3→P4a→P5→P6。
- P4b依P0连续协议/P3一致轨迹独立开展；P7依P2/P4a直接基准与大模型数据独立开展，均不作为核心P6前置。
- matrix_view/adams_wheel_center_12_v1在P0定契约，P2/P4a按请求支持，P5/P6按实际能力验收或明确拒绝；不额外增加强制核心任务。
- depends_on分号分隔，task_dir相对Epic根。共享契约/ABI/物理组装/结果/台账按依赖串行写；不在会话外并发修改同批文件。
- P0冻结native定义并如实登记Adams未知项即可推进native链；未知规则严格阻断Adams兼容路径/P5/P6，不能用native规则默认补齐。
- 子代理只读探索/核验，主代理负责修改/取舍/最终验证。规划必须独立审查并修订阻断项。

## 恢复与状态流程

1. 读本文件、SUBTASKS、父PROGRESS，再读首个依赖已DONE子任务的SPEC/TODO/PROGRESS。
2. 实施前完整阅读确切代码，核对最新源与接口/边界；旧任务DONE/历史状态不作为事实。
3. 每进入新步骤重读当前TODO并更新IN_PROGRESS，执行行为验收及规定门，记录真实证据。
4. 失败保留FAILED/retry_count，同类连续3次暂停重评；不得静默改弱断言/容差。范围策略改变记录并复核依赖。
5. 子任务全部步骤/行为/规定门通过才DONE，回填父CSV/恢复块；缺参考/实现/核心状态不得完成。
6. P6核心发布与整Epic完成分开：核心不等P4b/P7，整个Epic仍须9项全部DONE及适用终验通过。

## 设计覆盖与最终验收

| 要求 | 责任任务 | 判定 |
|---|---|---|
| §3七类指标、四组合、完整profile/冻结规则 | P0锁清单；P1几何/转向；P2平衡；P3时间；P4a动态局部 | P6全目标逐项，不删除失败项 |
| §4请求/case/reader/capability/result、身份/单位/版本 | P0全链契约；各阶段真实能力 | P6旧请求/无请求兼容及通道往返 |
| §5目标DAG/物理唯一归属、后端多RHS/转置 | P0现状；P2服务；P3/P4a复用 | AST/DAG及R7/R8证据 |
| §6.1–§7运动学/中心/轴、相容/测量唯一性 | P1统一算子；P2/P4a局部响应 | 解析/有限扰动/单位frame/奇异与退化专项 |
| §6.6–§6.7按需模式与Wheel/Ride/Roll/角度 | P2单一公式；P4a复用 | 直接/转置/全基可定义时等价；奇异组合单独验 |
| §8原接受态、初始/输出/接受步身份与只读状态 | P0预算；P3调度；P4a局部 | 核心完整一致状态、轨迹/事件/步序不变、不降采样 |
| §5.1/§10查询/实际CSV-Parquet/原生往返 | P1最小查询/JSON-NPZ；P6完整格式 | 禁求解探针、值/身份/单位/有效性/元数据保真 |
| §10 R1–R9与保留入口 | P0台账；P1 R1–R6；P2 R7/R9及R8定位；P3/P4a R8/R9 | P6真实调用迁完/唯一归属/旧值一致 |
| §11.1 Adams六组规则/独立参考 | P0审计；P5逐项对标 | pending阻断完整P5/P6，不以native验证替代 |
| 连续描述符/大型后端 | P4b/P7独立 | 各自真实能力/成本/误差验收，不成核心前置 |

## 公共质量门与证据

实施必跑just check-fast，内容以当前justfile为准（ruff/ty、架构脚本/附加旧模型测试、快速multibody及另外两包分开测试）。native改动先just build-kernel，求解/力导数变化追加just gate-numeric。
结构变化必须重跑三条AST脚本；P6大重构收尾再跑完整architecture目录及just test-all，日常不默认全量。独立扩展若跨系统收尾或准备提交，按AGENTS执行适用全量。
新增tests/characteristics/默认纳入快速集，kernel/contracts仍分开调用，不能混rootdir。SPEC中未来测试路径不是已存在/已运行证据。
文件存在/门禁命令退出0均不足以独自证明行为验收。各raw保存实际命令/退出码、解析/独立有限扰动步长与误差、模式/测量/预算/来源以及调用迁移证据；子任务记录复核结论。
AGENTS所列历史基线（1297 passed/1 skipped/1 xfailed；快速854 passed/1 xfailed、另两包59 passed）仅作参考，P0重测当前；skip/xfail不增长，不重录冻结数值/性能基线。
无请求无额外切线/输出，开启也不改原轨迹/事件/步序；同A共享分解、跨点重新数值分解。计量组装/分解/回代/序列化、峰值内存/体积和全接受步预算。

## 已知风险与确认边界

- Adams本地help不足以自动证明动态冻结规则；未知pending_definition、无独立参考pending_reference，核心缺项阻断P5/P6。商业软件/许可或外部数据生成不属本轮授权。
- 现有分解/单RHS不等于统一solve-many/transpose接口；P2真实实现/验收，P7收益/VJP也不预设。
- 现有CSV/Parquet读取不等于formats实际写表；P6补实行为，复用已有pyarrow/csv。
- R5旧对象映射、R8共享片段、核心输出一致状态是必须解决的实质缺口。
- 涉及结果/数据/范围的歧义、核心文件删除、新依赖、生产/真实数据/外部服务/商业资源、高风险Git或显著扩范围需按用户规则确认；本轮只写规划，不实施这些操作。

## 完成条件

- [ ] P6核心：四组合核心数值/P5兼容/查询导出/预算/R1–R9/收尾门全过，可发布所验范围的当前工作点SVC。
- [ ] P4b连续动态交付单独通过，未传播不声称时间响应。
- [ ] P7后端有正确性/稳定性/真实成本结论，是否推荐以证据判定。
- [ ] SUBTASKS九项全DONE且子TODO/证据一致、适用终验通过，才整个Epic complete。
