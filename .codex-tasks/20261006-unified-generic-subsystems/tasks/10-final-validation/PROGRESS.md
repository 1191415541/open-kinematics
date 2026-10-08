# 相关验收与文档交付进度

## 当前恢复块

- 当前步骤：3；状态：DONE；进度：3/3。
- 真源：TODO.csv。
- 验证：关键73项、完整架构295项、快速1553 passed/1 xfailed、完整Cases101项、kernel/contracts161项及数值三门通过；静态/结构五门和diff检查通过；全部叶机器证据在raw/validation。
- 下一步：无剩余实施任务；Git提交状态由git log核对。

## 执行记录

- 2026-10-08：删除后快速1552 passed/1 xfailed、kernel/contracts161 passed、完整架构295 passed。首次cases100 passed/1 failed，原MatchReport字段引用已修复并定向验收；完整cases待最终复验。
- 2026-10-08：独立核验发现整车轮胎测试只走轴模型，以及退役workflow模块未登记。整车测试改为四轮/29刚体普通装配，显式轮胎质量和惯量矩阵进入统一契约；补齐workflow防复活门。新增整车回归发现离线迁移路面摩擦系数时覆盖掉质量属性，修复为保留全部属性且仅更新轮胎力律；审查修复集158项通过。
- 2026-10-08：通用IR保留显式零质量/惯量；原仅检查旧emitter默认字段缺省的断言改为无惯性贡献断言。冻结结果字节等价另由26份原产物哈希和真实产物回归保障。实体迁移测试新增joint/tire数量、轮胎绑定体、质量及惯量逐项核对。
- 2026-10-08：单一路径核验未发现第二条实体构建/准备/解码链。compile_resolved默认protocol是Study的native激励协议选择，始终调用同一实体emitter与runner；它不按assembly业务角色构建模型。保持已有Study可省略protocol的契约。
- 2026-10-08：用户要求完成后提交全部更改并使用中文提交说明；验收和任务真源收拢后执行Git提交。
- 2026-10-08：最终数值复验通过。26份冻结产物组合SHA256=fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9；八族门通过；整车5 STRICT和3 PHYSICAL_DIFFERENCE保持分列；K100 best1.1684s/C66 best1.8134s均在原预算内。原9项自收敛FAILED及真实Adams证据缺失BLOCKED保留，不宣称这些旧验收通过。
- 2026-10-08：整车轮胎增加非默认道路摩擦系数0.5回归，同时检查质量5kg及其它三胎零质量；真实迁移和统一编译5项通过。
- 2026-10-08：提交前完整快速1553 passed/1 xfailed；ruff/ty、退役AST、内核分层、五个文档示例和diff检查通过。9份库存冻结基线文件SHA256逐项不变；全部测试收集通过。
- 2026-10-08：完整Cases复跑在test_the_mode_redistributes_the_same_travel触发native进程Aborted，尚不能视为无关或已验收。单独实际运行该用例1 passed/266.86s，未修改产品代码或断言；正串行复跑完整Cases并捕获stdout/stderr。首次abort保持在raw/cases-precommit.log及本记录，禁止以定向通过代替完整相关回归。
- 2026-10-08：完整Cases串行复验101 passed/539.03s，无skip/xfail；保留此前abort事实，单独与完整复跑均未复现，未为此修改产品代码、删断言或豁免测试。最后closeout五项静态/结构门及git diff检查通过。全部2167项收集成功；仅跳过无关Adams力律慢对标，不将收集成功表述为运行全量。
