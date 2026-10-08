# 唯一解析编译与研究计划进度

## 当前恢复块

- 当前步骤：4；状态：DONE；进度：4/4。
- 真源：TODO.csv。
- 验证：四组实际42/43/50/97项通过；补充spin与case路径10项通过；ruff/ty、三个结构门和数值门通过。日志见raw/validation/child-7-step-*.log/json及raw/numeric.json。26个产物SHA为fdfd5a6ba50970571ac31eb278cf5c713964a43ba77cd74fc1011ec8651eebc9；原9个自收敛失败与Adams缺证据保持登记。
- 实施：compilation/resolved.py统一模型/运行计划发射；compile_generic与ResolvedModel请求汇入同一编译器。模型保留不可变TIR二进制资源，SolvePlan保留工况激励而不复制模型。native追加motion能力公告；继续核验全工况和validate。
- 修复记录：首次新测试把已有零尾字节重写为零、C载荷使用错误字段、读取错误omega槽，均修正测试输入；实际旧case允许无name blob，编译器已兼容该契约字段。
- 边界：只读计划审查确认7/8与9的验收循环，已限定7/8为显式新路径就绪。旧生产链按任务9退出门要求暂未切换删除；全局唯一生产路由的最终验收不变。
