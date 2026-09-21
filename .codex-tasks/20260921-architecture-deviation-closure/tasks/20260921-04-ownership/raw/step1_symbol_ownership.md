# 步骤 1：mb_vehicle / mb_suspension 逐符号归属表（冻结）

来源：`cpp/src/vehicle/*.cpp`（6 TU，3230 行）、`cpp/src/suspension/*.cpp`（4 TU，448 行）、`cpp/src/abi/kernel_model_build.cpp`、`cpp/src/model/kernel_model.cpp` 全量阅读。归属依据 Epic 迁移矩阵与 02 冻结的 MIGRATION_MAP。

## → mb_assembly（装配：注册 + build_model + element reader）

### 车辆注册 11 函数（kernel_registration.cpp → src/assembly/registration.cpp）
- add_vehicle_steering_actuators (:25)
- add_driven_coordinates (:155)
- add_vehicle_aerodynamic_drags (:252)
- add_vehicle_road_profile (:307)
- add_vehicle_static_rotation_gauges (:355)
- add_vehicle_tire_frames (:426)
- add_vehicle_tire_models (:467)
- add_vehicle_drive_torque_mappings (:638)
- add_vehicle_spring_curves (:694)
- add_vehicle_bushing_curves (:805)
- add_vehicle_bushing_rotation_coordinates (:891)

### build_model 与 element reader（abi/kernel_model_build.cpp → src/assembly/）
- `build_model` (:25)：整个 TU 的核心；调用 11 个注册函数。
- `audit_constraint_system` 调用点 (:378)：审计函数本体已在 `mb_solve_static`（kernel_static_contact.cpp:638）——它读取 Model 并验证约束系统，失败时机与诊断保持不变。SPEC 允许"joint 审计或 ABI 装配后校验"；**维持现状（调用点在 build_model 内，本体在 solve_static）不动**：搬迁本体到 mb_joint 会引入 assembly→solve_static 反向边风险，而 SPEC 禁止该边。审计本体位置随 04 收尾的环消除需要再评估（见下）。

### element reader
- `read_element_blocks`：kernel_model.cpp 的注释指明它在 `kernel_model.cpp`（model 目录）；ABI `element_reader.cpp` 是另一层读取。**mb_model 纯数据化**（验收 4）要求它迁出 model：`read_element_blocks` 归 `mb_assembly`。

### mb_model 其他需迁出符号（纯数据化）
- `add_force_on_body`、`add_torque_on_body`、`mat6_mul`、`bushing_deformation`（kernel_model.cpp:18-88）：力装配数学 → `mb_force`
- `road_profile_height`、`road_profile_slope`（kernel_model.cpp:89-126）：路面查询 → 裁定：路面是中性输入数据访问，留在 mb_model（纯数据边界允许数据访问器）
- 质量矩阵装配（验收 4"质量装配归 mb_assembly、线性分解归 mb_linear"）：查 kernel_model.cpp 无质量装配函数——质量块在注册函数内构建（add_vehicle_tire_models 等含质量参数），归 assembly 随注册函数迁移。

## → mb_element（力元本构：标量与方向导数同迁，不拆散）

### 悬架（src/suspension/ → src/element/，整目录）
- assemble_spring_forces (spring.cpp)
- assemble_bushing_forces (bushing.cpp)
- assemble_anti_roll_forces (anti_roll.cpp)
- bushing_curve_value_slope, integrate_bushing_curve_from_zero (curves.cpp)

### 转向/驱动/制动（vehicle/steering.cpp + drive_brake.cpp → src/element/）
- assemble_steering_forces (steering.cpp)
- assemble_drive_brake_torques (drive_brake.cpp)

### 气动力元方向导数（kernel_directional.cpp:24 external_force_aerodynamic_directional）
- external_force_aerodynamic_directional → mb_element（气动力元本构的方向导数）

### 方向力元（kernel_directional.cpp，同一文件的元件本构部分 → src/element/directional.cpp）
- external_force_spring_directional (:72)
- external_force_bushing_directional (:205)
- external_force_anti_roll_directional (:333)
- external_force_steering_directional (:393)
- assemble_directional_elements (:571)
- assemble_directional_drive_torques (:619)
- write_directional_detachment (:949)

## → mb_force（两条总线 + 外力重力 + 广义力汇总）

### 总线（vehicle/force_assembly.cpp + layout.cpp → src/force/）
- external_force_vector（force_assembly.cpp:17，标量总线）
- external_force_directional（kernel_directional.cpp:1595，对偶总线）→ mb_force
- assemble_external_and_gravity (layout.cpp:109)
- assemble_generalized_force (layout.cpp:193)
- assemble_aerodynamic_force (layout.cpp:26)：外部合力计算 → mb_force
- reset_force_outputs (layout.cpp:63)：输出清零 → mb_force
- add_force_on_body / add_torque_on_body / mat6_mul / bushing_deformation（从 model 迁入）
- assemble_directional_generalized_force (kernel_directional.cpp:1549)：对偶广义力汇总 → mb_force

## → mb_tire（kernel_directional.cpp 的轮胎语义 → src/tire/）

- directional_tire_frame (:751)
- directional_body_origin (:968)
- directional_contact_arm (:980)
- assemble_directional_fiala_slips (:988)
- apply_static_contact_directional (:1044)
- assemble_directional_pac2002_force (:1129)
- apply_directional_fiala_force (:1400)
- apply_directional_brush_force (:1476)

## 环消除裁定（关键）

03 收尾后的环：`mb_solve_dynamic <-> mb_solve_static <-> mb_tire* <-> mb_vehicle`。
拆解 mb_vehicle 后该环的边全部重定向到 assembly/force/element/tire。
- `mb_solve_static <-> mb_vehicle` mutual 边：static 调 vehicle 的力装配（→ mb_force）与 build_model（→ mb_assembly）；vehicle 调 static 的 audit_constraint_system 与 pose 相关。拆解后 `solve_static -> {force, assembly}` 正向、`assembly -> joint/model` 正向——环应消除。
- `mb_solve_dynamic <-> mb_tire_state` mutual 边：属 03 遗留，dynamic 调 tire_state 的状态读写、tire_state 调 dynamic 的 tire_block_width/read_tire_states。**本任务范围外**（mb_tire_state 是 kept 模块）；若终局仍剩此 mutual 边，登记为 08 前的收尾项。

## 凝聚交接（05 的输入，本任务只登记）

- 凝聚计算来源：`packages/suspension_multibody/src/suspension_multibody/model/`（Python 侧焊接体凝聚，01 冻结的 SYMBOL_MATRIX 已列 owner=05）。
- 需求：native 装配（mb_assembly）后续需提供原 body ID → 凝聚体 ID 映射通道；身份映射实现与验证统一由 05 负责，本任务不实施、不宣称通过。

## 文件重定向总表

| 现路径 | 新模块 | 新路径 |
|---|---|---|
| src/vehicle/kernel_registration.cpp | mb_assembly | src/assembly/registration.cpp |
| src/abi/kernel_model_build.cpp（build_model 部分） | mb_assembly | src/assembly/build_model.cpp |
| src/vehicle/layout.cpp | mb_force | src/force/layout.cpp |
| src/vehicle/force_assembly.cpp | mb_force | src/force/external_vector.cpp |
| src/vehicle/steering.cpp | mb_element | src/element/steering.cpp |
| src/vehicle/drive_brake.cpp | mb_element | src/element/drive_brake.cpp |
| src/vehicle/kernel_directional.cpp | 拆三分 | element/directional.cpp（元件）、force/directional.cpp（总线+广义力）、tire/directional.cpp（轮胎语义） |
| src/suspension/ 整目录 | mb_element | src/element/ |
| read_element_blocks（kernel_model.cpp） | mb_assembly | src/assembly/element_reader.cpp |
