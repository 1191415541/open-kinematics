# 通用关节坐标与车轮自转边界进度

## 当前恢复块

- 当前步骤：4；状态：DONE；进度：4/4。
- 真源：TODO.csv。
- 验证：child-5日志13/3/20/15项通过；ruff、ty和三个结构门通过；数值门3项退出码0，旧动态逐位一致8族parity和性能预算通过。原有9个自收敛失败及Adams实证BLOCKED保持task1库存记录，无基线重录。
- 交付：Coordinate/Spin Port、Motion边界解析、native标量旋转残差及Scalar/Dual、平面道路校验与K/C旋转初值修正；锁止反力、拒绝矛盾初速、free自转及18rad多周驱动已实测。
- 失败记录：首次native构建用了错误Dual类型名，已改DirectionalScalar；probe与reader缺numeric函数头，已补；两次测试在镜像构建完成前执行触发stale mirror，完成构建后通过；道路参数误校验已修复。
- 下一步：task6核验native有状态力元accepted-step与拒步重放；motion解析由task7接入唯一compiler。
