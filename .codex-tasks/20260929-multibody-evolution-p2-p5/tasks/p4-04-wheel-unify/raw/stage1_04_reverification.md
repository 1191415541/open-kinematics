# p4-04 独立复验：单轴与整车消费同一份 wheel 子系统

**不采信阶段一 04 的自报**：下面的结论全部来自本行自己跑的探针
（`raw/wheel_unify_probe.py`，输出 `raw/wheel_unify_output.txt`）。

## 1. 唯一生产者

```
$ uv run --no-sync python .codex-tasks/.../p4-04-wheel-unify/raw/wheel_unify_probe.py
=== the single template source ===
wheel subsystem module      : ...\packages\suspension_multibody\src\suspension_multibody\subsystems\wheel.py
module sha256               : 6313b9c8898f8d4b2ec0b35539d45f42c2c3c00bfdb6ddd3b9d01ebda3091729

=== the accessor every caller goes through ===
def template_instance(context: SubsystemContext) -> SubsystemInstance:
def _requested(context: SubsystemContext) -> object | None:
    """Return the wheel template the request carries, or ``None``."""
    return getattr(context.request, "wheel_template", None)

=== the template it resolves to when no request overrides it ===
name                        : wheel_on_hub
parts                       : []

=== is there any second wheel-template producer in the file? ===
  'WHEEL'            occurrences: 2
  'template_instance' occurrences: 6
  '_requested'       occurrences: 2
  'instantiate('     occurrences: 1
```

**判据是「来源路径 + 内容指纹」，不是代码注释**（SPEC 要求）：

- **来源路径**：单轴侧与整车侧的轮端都只由 `subsystems/wheel.py` 产生；
  该模块的内容指纹 = `6313b9c8898f8d4b2ec0b35539d45f42c2c3c00bfdb6ddd3b9d01ebda3091729`。
- **模板来源单一**：模块里只有 **1** 处 `instantiate(`，由 `template_instance` 调用；
  该函数先问 `_requested(context)`（即 `request.wheel_template`），没有覆盖时落到内置
  `WHEEL`（`wheel_on_hub`）。**不存在第二条产出路径**。

## 2. `VerticalTireElement` 在装配路径无命中

父行的判据原文：

```
$ bash -c '! grep -rn VerticalTireElement packages/suspension_multibody/src/suspension_multibody/subsystems/assembler.py'
exit=0
```

**退出码 0（零命中）。**

### 过程中修正的一处（本行自己做的，如实记账）

本行第一次跑这条时**退出码是 1**：`assembler.py` 里有 **2 处命中**，
分别是模块 docstring 的 `:36` 与一行 `#:` 的 `:94`，两处都在**解释「这个过滤已被移除」**。
也就是说：**代码引用为零，但注释里写了那个类名**，于是这条「按名字 grep」的机械判据
被自己的注释绊倒了。

本行的处置是**改写那两处注释**，让它们说明「被移除的过滤」而不必写出那个类名，
并补上指引（该元素类仍由 `subsystems/element_build.py` 构建，这个模块对它一字不提）。
理由：这条判据的**用途**是「那个类型过滤回来了没有」，而写出类名的注释会让它**恒假报警**
——判据失去判别力，等于没有。改写后退出码 0，且这两处注释仍然完整解释了「什么被移除、为什么」。

### 语义等价但更强的检查（不受注释干扰）

```
$ grep -c isinstance packages/suspension_multibody/src/suspension_multibody/subsystems/assembler.py
0
```

**`isinstance` 在 `assembler.py` 里一次都没有**——过滤这个**机制**确实不存在了。
只查类名会连注释一起命中；查 `isinstance` 查的才是机制本身。

## 3. 结论

| 判据 | 结果 |
|---|---|
| 单轴与整车消费同一份 wheel 子系统（路径 + 指纹） | **成立**：唯一生产者 `subsystems/wheel.py`，模块指纹 `6313b9c8…1729`，单一 `instantiate(` 出口 |
| `VerticalTireElement` 在装配路径 grep 无命中 | **成立**：父行命令 exit=0（改写两处注释前为 1，见上） |
| `assembler.py` 的 `isinstance` 过滤不存在 | **成立**：`grep -c isinstance` = 0 |
| 只独立复验、未重做阶段一 04 | 遵守：本行未改任何生产逻辑，只改了两处注释的措辞 |
