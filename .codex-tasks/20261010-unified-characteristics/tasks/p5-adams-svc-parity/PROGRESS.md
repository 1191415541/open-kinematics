# P5 实施进度

## 恢复入口

- 日期：2026-10-13（Asia/Shanghai）。
- 任务：用独立 Adams Car 2024.1 参考逐样本对标核心四组合局部响应与指标，仅证据齐全且误差通过的条目变为 validated。
- 形态：single-full；状态：**IN_PROGRESS**；软件/许可阻断已解除，定义审计已部分收窄，独立参考仍缺。
- 进度：**1/5 步 DONE**；步骤 1 DONE（2026-10-13，Definition-审计重建）；步骤 2–5 TODO。P5 整体未完成（核心逐点对标未做）。
- 真源：[TODO.csv](TODO.csv)；范围：[SPEC.md](SPEC.md)；父真源：[SUBTASKS.csv](../../SUBTASKS.csv)。
- **2026-10-13 步骤 1 重做（推翻旧 FAILED 前提）**：旧记录的"本机无 Adams Car 2024.1"**已被实测证伪**。
  - 实证：`C:\Program Files\MSC.Software\Adams\2024_1` 存在（`bin\adams2024_1.bat`、`acar\shared_car_database.cdb` 齐备）；
    项目自带 `probe_profile('adams-car-2024.1')` 返回 `available=True / version=2024.1 / license_probe=passed`；
    `validate-adams --smoke --require-installed` **exit 0**。
  - 旧 FAILED 根因：落点判定命令检查 `tasks/p5-adams-svc-parity/raw/adams-rule-audit.csv`（该文件当时从未创建，旧审计写在 P0 目录），
    且 `external-blocker.md` 的四条路径核对**漏掉了真实的 `C:\Program Files\MSC.Software\Adams\2024_1`**。属路径范围不足的假阻断。
  - 交付：[raw/adams-rule-audit.csv](raw/adams-rule-audit.csv)（2024.1 六组规则审计，含逐字引文与实测 SHA256）+
    [raw/adams-rule-audit.md](raw/adams-rule-audit.md)（环境事实、页面映射差异、复核链）。
  - 结论更新：**G6 `stated`**（`[C]=[dX/dF]`、12×12 左右轮心布局、`C(3,3)/C(3,9)/C(9,3)/C(9,9)` 索引、
    `wheel_rate.left=1/(C33+C39)`、ride rate 串联式、roll rate 轮心 vs 接地点 `+2/Kt`、角度 `(pi/180)`，
    全部在 `example_o_wheelenv.html`/`svc_d_5.html`/`svc_wheel_rate.html`/`svc_ride_rate.html`/`svc_roll_rate_*.html` 逐字可引）；
    **G3 `partial` 但关键结论翻转**——`adams_solver/gtcmat.html` 明确给出 C++ Solver 动态柔度算法
    （线性化不改当前状态 → 输入输出表示 → 速度/加速度扰动项可忽略，动态或准静态均可调用），
    旧审计"动态段不给出完整算法"对 Solver 手册**不成立**；仍缺"冻结何处/何参考系 wrench、陀螺项"。
    G1/G2/G5 维持 `partial`（有部分逐字出处，关键项仍缺）；G4 维持 `not_stated`。
  - 匹配实验（既有本地产物，非本轮新跑）：`artifacts/adams/strict-k-real`（9×90 field）与
    `artifacts/adams/strict-c-real`（66×1056 field）**failed=0**，最大绝对误差 1.837e-04 / 4.079e-09，
    执行证据自述 `adams_version=2024.1`。**属既有静态/准静态严格门，不等于 P5 要求的 SVC 逐项对标，不得替代。**
- 已完成：步骤 1 定义审计（2024.1 官方 help 重建 + 指纹 + 匹配实验登记）；旧外部阻断前提证伪与根因定位。
- 启动依赖：P0;P1;P2;P3;P4a；P4a 当前 IN_PROGRESS（步骤 6 FAILED），P5 步骤 3–5 的逐点对标仍待 P4a 核心响应可产出。
- 下一步：步骤 2 整理独立参考及同模型采样映射（需与统一特性请求同模型/边界/冻结协议的独立 Adams 参考，当前 `pending_reference`）；
  之前先复核 P4a 能否产出可与 Adams 逐点比较的核心响应。
- 最新实施验证：`probe_profile` available=True；`validate-adams --smoke --require-installed` exit 0；
  19 个 2024.1 help 引用文件 SHA256 实测（见 CSV `source_sha256` 列）；既有对标报告 JSON 由主代理直接解析（未采信二手转述）。
  参考仍保持 `pending_reference`；未伪造任何 Adams 运行或误差报告。

## 证据与同步规则

实施在raw/保存实际命令/退出码、oracle/误差、预算测量/调用迁移证据。未通过不DONE；同类3次失败暂停重评，保留FAILED/retry_count。每步同步本恢复块；子任务完成后回填父SUBTASKS，completed_at记实际时间。
