# 2026-10-08 最终交付证据

全部10个子任务、34个叶步骤完成；每个DONE叶步骤都有raw/validation中的命令、退出码、时间及日志SHA256。删除前门原始执行单独保存在cutover-validation.json；步骤9-2以只读档案校验生成统一格式记录，不重演已经删除的源码或补造历史时间。

生产路径统一为声明文档 → DocumentLoader → 通用装配/ResolvedModel → compile_resolved → NativeBackend → ResultEnvelope。文件/Python对象、悬架/整车/普通机构共享这一链；旧执行模块和公开导出退役，v1只保留离线迁移。Wheel内部拥有轮体和Tire；Brake/Drive、Rig使用同样的声明单元。同一Wheel/Suspension定义由Case边界决定locked/free/prescribed spin，轮胎contact frame引用非自转carrier。

| 验证 | 结果 | 原始证据 |
|---|---|---|
| 完整快速集（排除adams/architecture/cases） | 1553 passed，1既有xfailed | fast-precommit.log |
| kernel/contracts独立调用 | 161 passed | kernel-contracts-final.log |
| 完整架构（退役后） | 295 passed；workflow新增门禁另在158项中通过 | architecture-after-retirement.log、final-audit-fixes.log |
| 完整Cases串行复验 | 101 passed，539.03s | cases-serial-final.log |
| 唯一路径/文件Python/spin关键集 | 73 passed | validation/child-10-step-1.log |
| 最后属性/实体迁移/退役/冻结产物修复集 | 158 passed；新增非默认摩擦5项进入最终快速集 | final-audit-fixes.log、fast-precommit.log |
| 数值三门 | 26产物字节等价、八族验收、K/C性能预算通过 | validation/child-10-step-2.log |
| ruff/ty、三项结构门、diff检查 | 全部退出0，AST findings=0 | validation/child-10-step-3.log |
| 文档/文件示例 | 五例通过；轮荷205.93965N、总质量21kg、反馈转速-0.0588165261rad/s | closeout门日志、examples/generic_multibody/verify.py实际运行 |
| 冻结文件完整性 | 库存9份原始文件SHA256全不变 | frozen-baseline-final.json |
| 全部测试收集 | 2167项可收集，不代表运行全量 | collection-precommit.log |

数值边界：整车5项原哈希STRICT；3项批准的contact frame修正为PHYSICAL_DIFFERENCE，保留原严格不匹配并通过物理门。26产物组合SHA256为fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9。原9项自收敛FAILED和真实Adams缺证据BLOCKED没有被改为通过。本次未运行无关Adams慢力律对标，未新增skip/xfail或重录冻结基线。

过程异常：完整Cases首次复跑在左右反向整车轮跳触发native Aborted；单例实际运行1 passed/266.86s及完整串行101 passed均未复现。没有为此修改产品代码、削弱断言或豁免测试；保留cases-precommit.log、vehicle-kc-abort-repro.log和执行记录，原因未确定。
