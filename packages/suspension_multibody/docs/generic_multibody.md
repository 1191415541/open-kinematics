# Python 纯数据多体入口

`generic_multibody` 装配直接解释模板的 `bodies`、`joints`、`elements` 和端口，生成原生 `multibody-model`，再由已有 C++ 内核求解。它不经过悬架 `AxleDeclaration`，也不调用七类业务构建器。台架作为普通子系统列在同一 `subsystems` 表内。

文件示例位于 `examples/generic_multibody/`：滑块、弹簧、阻尼器的平衡运行，以及普通模板台架通过端口驱动滑块的行程扫描。

```python
import json
from pathlib import Path
from suspension_multibody import simulate

root = Path("packages/suspension_multibody/examples/generic_multibody")
case = json.loads((root / "dynamic.case.json").read_text(encoding="utf-8"))
run = simulate(root / "assembly.json", case)
assert run.status == "success"
z = run.result.body_state("slider.sub.json.carriage")[:, 2]

case = json.loads((root / "kinematics.case.json").read_text(encoding="utf-8"))
run = simulate(root / "bench.assembly.json", case)
```

声明规则：

- 模板/子系统角色为 `generic`，装配类别为 `generic_multibody`。装配的 `mode` 选择 `K` 或 `C`；关节、元素和端口连接的 `modes` 筛选激活实体，关节的 `kind_by_mode` 选择类型。K/C 切换不移动几何，也不按业务角色猜测拓扑。
- `units.length` 为 `m` 或 `mm`，省略时沿用作者层的 `mm`。硬点为世界坐标；刚体 `position` 为初始参考原点，`quaternion` 为标量在前的姿态；质量为 kg，惯量为 kg 乘以长度单位的平方。`center_of_mass` 在刚体局部坐标内，编译时将原生刚体原点移到质心，硬点位置保持不变。
- 关节独立声明两端硬点，允许同一硬点承载多个关节。`axis`、`axis_b` 是世界方向；`axis_reference` 从 `point_a` 指向另一硬点。万向节必须给两根轴。缺少必要轴直接报错。
- `symmetry: mirrored_xz` 生成 `_L`/`_R` 实体，标准反射处理位置、四元数、惯量和极/轴向量。公共支撑用 `symmetric: false` 保留一个实体；`asymmetric` 不复制。旧 `sides`/`mirror` 声明继续有效，显式 `symmetry` 与 `mirror` 必须一致。`sides: [left, right]` 配合 `mirror: false` 表示两侧实体已分别写在数据里，解释器保留各自名字和几何。
- 弹簧、阻尼器、限位器和衬套读取已解析的属性文件，分别按文件单位转换到 SI。元素 `parameters` 使用模板的长度单位；六轴衬套矩阵按平移、转动及耦合块分别换算。扭转弹簧使用原生 `anti_roll_bar` 类型和显式局部 `axis_a`，不是旧悬架防倾杆的轮端位移力律。
- 端口必须明确 `owner` 和 `point`。连接表使用 `port_a`、`port_b`、`type`，轴和衬套参数按需要声明；装配级衬套连接的 `units.length` 默认 `m`，可显式为 `mm`。显式绑定也校验角色、能力和标签；匹配结果保留在 `bindings`，缺失可选分支会移除其 `bound_outputs` 声明。匹配只解决邻居归属；`fixed` 生成固定副，保留各刚体质量。几何重合不会自动焊接或合并刚体。
- 所有已支持的时间历程和网格工况通过同一 `ResolvedModel` 编译；工况名称是内核协议键，不决定模型实体。K 扫描用 `k.axes` 指定任意驱动坐标；C 载荷的 `load_marker` 使用明确 frame 全名。结果读 `run.result`，通过稳定实体 ID 查询状态、wrench 和声明测量。

采样输入由 `CaseDocument` 和 `ResolvedSolvePlan` 显式声明，交给 `compile_resolved` 与 `run_compiled`。Wheel 子系统声明轮体和 `tires` 复合单元，Brake/Drive 子系统声明公式力元；旧输入仅由 `authoring.migration` 离线转换。公开入口的最终退役由删除前退出门约束。

文件级轮胎和函数力元例子可直接运行：

```bash
uv run --no-sync python packages/suspension_multibody/examples/generic_multibody/verify.py
```

它会验证 TIR 文件绑定、唯一轮体质量（20 kg 轮体加 1 kg 承载体）、静态垂向载荷，以及读取当前相对角速度的有符号制动/驱动转矩。

原生程序测量覆盖 1、16、64 个表达式，每组 20,000 次求值、5 次取中位数，排除解析和进程启动。当前机器常量公式每次 scalar 约 170–178 ns、方向导数约 37–42 ns；含旋转 frame 相对速度反馈及 `tanh` 的公式分别约 318–333 ns、286–302 ns。这是观测成本，不作为跨机器固定阈值；旧模型仍由既有 K/C 性能门守护。
