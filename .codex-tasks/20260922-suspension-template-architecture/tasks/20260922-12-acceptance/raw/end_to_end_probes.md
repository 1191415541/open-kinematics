# 端到端九件事 (a)-(i) 的独立探针记录

探针脚本写在会话 scratch（`acceptance/probe_a_b_c.py`、`probe_d_e_f.py`、`probe_g_h_i.py`），
不落工作区；下面是逐条实跑输出原文。

## probe_a_b_c — (a) 同一模板 K/C，(b) 同一模板两 study，(c) 换属性文件

```
(a) K: 13 constraints / 0 bushings; C: 9 / 8 -> template-only difference OK
(b) both studies share one assembly object; studies = ('quasi_static', 'dynamic')
(c) geometry identical across files; stiffness [0.0] -> [25000.0]
A/B/C PASS
```

## probe_d_e_f — (d) 组合与自定义输出，(e) 整车实验总成，(f) 两侧轮胎质量

```
(d) pairs: 7 | axle+kc drives: ['wheel_drive_L', 'wheel_drive_R', 'rack_drive'] | vehicle+four_post drives: ['road_height_fl', 'road_height_fr', 'road_height_rl', 'road_height_rr']
(d) custom derived output over minimum-unit outputs = 1600.0
(e) vehicle roles: ['brake', 'chassis', 'drive', 'steering', 'suspension', 'wheel'] | bodies: 23
(e) vehicle - axle = ['brake', 'drive'] | axle - vehicle = []
(f) axle tire entry mass: 5.0 | vehicle tire entry mass: 5.0
D/E/F PASS
```

## probe_g_h_i — (g) 无转向总成，(h) 简化→复杂替换，(i) 可用性矩阵

```
(g) no-steering: rack dropped = ('rack_drive',) | grid states 3 | rack in axis_map: False
(g) with-steering: rack present = True | grid states 9 | dropped: ()
(h) complex template bodies: ['caliper_L', 'rotor_L'] | simplified bodies: []
(i) axle roles: ['chassis', 'steering', 'suspension', 'wheel'] -> brake/drive present: set()
(i) vehicle roles: ['brake', 'chassis', 'drive', 'steering', 'suspension', 'wheel'] -> brake/drive present: {'brake', 'drive'}
G/H/I PASS
```

## 验收发现并修掉的缺口

`rigs/compose.py` 最初把试验台自身的激励坐标（`road_height`、`steering_wheel_angle`）
也当作「必须由总成提供」的坐标，于是 **`vehicle × ride_four_post` 直接报错**——
全部动态试验台配不上任何总成。已引入 `DriveSpec.from_assembly` 区分「谁提供这个坐标」：
总成侧的驱动坐标会被收缩，试验台自带的激励不会被要求。

这条缺口是**端到端验收独立于子任务自证**才发现的：12 项 `tests/rigs` 当时全绿，
因为它们只覆盖了总成侧的坐标。
