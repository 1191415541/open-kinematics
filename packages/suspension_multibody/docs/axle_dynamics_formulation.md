# 整轴动力学方程与数值求解

## 状态和坐标

内核只使用 SI。世界坐标是右手车辆坐标：`+X` 向后、`+Y` 向右、`+Z` 向上。每个自由刚体 `i` 的状态为：

```text
r_i       质心世界位置，m
Q_i       body -> world 单位四元数，标量在前
v_i       质心世界速度，m/s
omega_i   世界角速度，rad/s
a_i       质心世界加速度，m/s^2
alpha_i   世界角加速度，rad/s^2
z_i       轮胎刷毛等内部状态
```

Newton 变量使用 6 维切空间增量 `delta y_i=(delta r_i, delta theta_i)`，不把四元数四个分量当作独立自由度：

```text
R_i(new) = Exp([delta theta_i]x) R_i(old)
Qdot_i   = 0.5 * [0, omega_i] tensor_product Q_i
```

四元数仅在更新后做机器精度级归一化；范数误差超过 `1e-10` 直接拒步。

## 连续 DAE 方程

对所有自由刚体装配空间质量矩阵：

```text
M(q) a + h(q,v) - f_ext(q,v,z,t) - J(q,t)^T lambda = 0
Phi(q,t) = 0
J(q,t) v + Phi_t(q,t) = 0
```

`M` 包含质量和质心处惯量，`h` 包含世界系陀螺项以及由刚体姿态产生的偏置项。约束行使用最小独立形式：球铰 3、转动副 5、移动副 5、固定副 6。

### 约束 Jacobian

`J=dPhi/dy` 对世界切空间增量 `(delta r, delta theta)` 取闭式，其中位姿更新为
`r <- r + delta r`、`R <- Exp(delta theta) R`。用到的基本导数：

```text
d(R*p)/d(delta theta)      = -[R*p]x
d(a_hat)/d(delta theta)    = -[a_hat]x            (旋转保长，归一化与旋转可交换)
d(Log(Ra^T Rb))/d(theta_b) =  Jl^-1(phi) * Ra^T,  phi = Log(Ra^T Rb)
d(Log(Ra^T Rb))/d(theta_a) = -Jl^-1(phi) * Ra^T
Jl^-1(phi) = I - 0.5*[phi]x + c(|phi|)*[phi]x^2
c(t)       = 1/t^2 - (1+cos t)/(2*t*sin t),  t->0 时取 1/12 + t^2/720
```

转动副与移动副还需垂直参考系 `e1,e2` 的导数。`e1` 由固定参考向量 `g` 对轴正交化并
归一化得到，因此 `d(e1)` 需按投影与归一化两步链式展开；`e2=aa x e1` 的导数由叉积
乘积法则给出。残量与 Jacobian 必须共用同一个 `g` 的选择规则，否则会线性化不同的系。

`Jl^-1` 中 `[phi]x^2` 项的系数在 `|phi|` 小时发生灾难性相消，必须切换到级数。省略
该项会得到看似合理但错误的 Jacobian：实测在移动副上产生 3.3e-3 的偏差。

内核每次运行都在初始位姿上用中心差分校验解析 Jacobian（相对容差 1e-6），不一致即
拒绝运行。该校验每次运行只做一次，相对于它替代的数十万次解析求值可以忽略。

## GGL generalized-alpha 离散

采用 Chung-Hulbert 二阶参数：

```text
alpha_m = (2*rho_inf - 1)/(rho_inf + 1)
alpha_f = rho_inf/(rho_inf + 1)
gamma   = 0.5 - alpha_m + alpha_f
beta    = 0.25*(1 - alpha_m + alpha_f)^2
```

给定步长 `h`：

```text
Delta y_GA = h*v_n + h^2*((0.5-beta)*a_n + beta*a_(n+1))
v_(n+1)    = v_n + h*((1-gamma)*a_n + gamma*a_(n+1))
a_am       = (1-alpha_m)*a_(n+1) + alpha_m*a_n
v_af       = (1-alpha_f)*v_(n+1) + alpha_f*v_n
```

旋转中间状态沿 `Log(R_(n+1) R_n^T)` 插值。动力学在 `a_am`、`v_af` 和中间位姿处评估，位置和速度约束在 `n+1` 处评估。

为使位置和速度约束在同一 Newton 系统中闭合，采用 GGL 位置增广：

```text
R_q = Delta y - Delta y_GA - W*J(q_(n+1))^T*mu
R_v = v_(n+1) - v_n - h*((1-gamma)*a_n + gamma*a_(n+1))
R_d = M*a_am + h(q_af,v_af) - f(q_af,v_af,z_af,t_af) - J^T*lambda
R_phi = Phi(q_(n+1),t_(n+1))
R_psi = J(q_(n+1))*v_(n+1) + Phi_t(q_(n+1),t_(n+1))
```

`lambda` 是物理约束反力；`mu` 是量纲一致的数值位置增广乘子，不作为关节反力输出。`W` 是由质量和转动惯量形成的块对角逆尺度。

未知量为：

```text
(Delta y, v_(n+1), a_(n+1), lambda, mu, z_(n+1))
```

总维数 `18N+2m+nz`，与 `(R_q,R_v,R_d,R_phi,R_psi,R_z)` 完全相同。

## 内部状态和接触

刷毛状态使用 Jansen 一阶隐式 generalized-alpha：

```text
alpha_m_z = (3 - rho_inf)/(2*(1 + rho_inf))
alpha_f_z = 1/(1 + rho_inf)
gamma_z   = 0.5 + alpha_m_z - alpha_f_z
```

一阶格式的权重加在**新**值上，与二阶 Chung-Hulbert 的约定相反：

```text
z_dot_(n+1) = (z_(n+1) - z_n)/(gamma_z*h) - ((1-gamma_z)/gamma_z)*z_dot_n
R_z = alpha_m_z*z_dot_(n+1) + (1-alpha_m_z)*z_dot_n - g(z_afz, q_afz, v_afz, t_afz)
z_afz = alpha_f_z*z_(n+1) + (1-alpha_f_z)*z_n,   t_afz = t_n + alpha_f_z*h
```

把二阶约定误用到一阶格式会得到与 `z` 无关、谱半径恒为 1.571 的放大矩阵，即无条件不稳定；
且 `rho_inf=1` 时两种约定重合（`alpha_m_z=alpha_f_z=0.5`），因此只在 `rho_inf<1` 暴露。
回归测试必须覆盖 `rho_inf<1`。

法向接触使用单边柔顺力：

```text
delta = max(0,-gap)
Fn = max(0, Felastic(delta) + cn*delta_dot)
```

不施加冲量和速度覆盖；接触进入/退出由 `gap=0` 和闭合方向定位事件。路面输入必须位置连续、速度有界。

切向刷毛状态（`u` 为接地点滑移速度，正方向与轮胎接触坐标一致）：

```text
s_dot = u - |Vroll|*diag(1/Lx,1/Ly)*s
Ftrial = -diag(Cx,Cy)*s
```

超过摩擦椭圆后径向返回并投影刷毛状态，塑性滑移功率必须非负。因此饱和后切向力必须与滑移
反向，`F dot u <= 0` 是回归判据。

## 一致初始化

先解静态配平 `(q0,lambda0,z0,contact_set)`，再由速度约束和加速度级方程求 `(v0,a0,lambda_a)`。静止工况 `v0=0`；滚动工况由纯滚动条件确定轮速。任何约束、接触、能量或内部状态残量超限都禁止进入瞬态。

### 静力学零影响方向

经转动副连接的车轮，其自转角对静力学完全无影响：重力作用于质心、轮胎法向力沿法向作用于
接地点（对自转轴无力矩）、弹簧接在轮心。该坐标在静态 Jacobian 上的**行与列同时恒为零**，
配平矩阵必然奇异。处理规则是：仅当某位姿坐标对所有残量都无影响（行列均在容差内为零）
**且**其自身残量已满足时，才把该坐标钉在初值；钉住的方向数写入诊断列
`static_trim_pinned_null_directions`。这不是对可辨识方向的正则化——真正秩亏的约束集仍被拒绝。

## 能量审计

每个接受步记录：

```text
Delta(T+V+E_elastic)
  = W_external + W_road + W_drive_constraint
  - D_damper - D_friction - D_contact - D_algorithm
  + R_energy
```

轮胎法向/刷毛、稳定杆、限位储能和时变约束功单独列项。能量归一化分母由 `ACCEPTANCE.yaml` 的 `energy_normalization` 冻结。

## 失败策略

Newton、线性求解、主动集或能量门失败时拒步并二分；每次拒步保存候选状态、残量、主动集和步长。达到最小步长仍失败则返回结构化失败结果，禁止跳过采样点或伪造收敛。
