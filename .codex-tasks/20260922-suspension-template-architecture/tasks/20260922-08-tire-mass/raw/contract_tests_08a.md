# 契约三例（08 前半）

文件：`packages/suspension_contracts/tests/test_multibody_contract.py`
schema：`packages/suspension_contracts/src/suspension_contracts/contracts/multibody_model.schema.json`
`$defs.tire` 新增可选 `mass`（number, minimum 0）与 `inertia`（3x3 数值数组，`minItems/maxItems: 3`）。

| 用例 | 断言 |
|---|---|
| `test_tire_validation_accepts_an_entry_with_mass_and_inertia` | 带 mass=12.5、inertia 对角矩阵的 tire entry 被接受（validate_model 不抛） |
| `test_tire_validation_rejects_a_malformed_mass_by_name` | `mass="heavy"` → 报错含 `$/tires[0]/mass` 与 `expected number` |
| `test_tire_validation_rejects_a_negative_or_non_finite_mass_by_name` | mass ∈ {-1.0, inf, nan} → 报错含 `$/tires[0]/mass` 且 `must be >= 0` 或 `must be finite` |
| `test_tire_validation_rejects_a_malformed_inertia_by_name` | 形状不足/超出 3、非数值项、负值四项 → 报错含 `$/tires[0]/inertia` 且点名具体原因 |
| `test_tire_validation_rejects_an_unknown_tire_field_by_name` | 未知字段 `unsprung_mass` → 报错含 `$/tires[0]` 与字段名（`additionalProperties: false` 仍生效） |

`multibody.py` 的 `_check()` 新增 `"finite": true` 关键字支持（number 分支），在 `minimum` 之前判定；
形状由既有的 `minItems/maxItems` 与 `items` 递归检查覆盖。

## 内核侧用例（`packages/suspension_kernel/tests/test_tire_mass.py`）

| 用例 | 断言 |
|---|---|
| `test_a_declared_mass_and_inertia_reach_the_built_tire` | build_model + install_tire_mass 之后 `Tire.mass == 12.5`、3x3 惯性 9 项逐项相等，且 `ContractModel` 解析表一致 |
| `test_a_document_without_the_fields_gets_zero_mass` | 不带 mass/inertia 的既有文档得到 mass 0 与全零惯量（既有行为） |
| `test_a_length_mismatch_is_named_rather_than_truncated` | 1 个 tire 的声明装到 0 个 tire 的模型上 → 报错点名 `1 masses / 1 inertias / 0 tires` |
| `test_the_appended_fields_left_every_earlier_tire_offset_alone` | 32 个既有字段偏移与 `sizeof` 与改动前实测值逐一相等 |
| `test_the_abi_version_constants_are_unchanged` | 版本常量 15/30/1 |
| `test_the_frozen_input_structures_are_untouched` | AxleInput/VehicleInput 字段数 106/97 未变（未追加数组字段） |
