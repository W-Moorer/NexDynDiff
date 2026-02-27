# 工具

此目录包含辅助脚本，用于将 RMD 场景转换为项目场景文件、运行基础接触回归测试，以及比较时序输出。

## 要求

- Python 3.8 或更高版本（脚本使用标准库）。

## 脚本

### rmd_to_scene.py

- **用途**：将 .rmd 场景文件转换为项目的场景格式，并导出 OBJ 网格 + 法线到 `models/obj/{scene_name}`。
- **当前已对齐的物理/仿真信息**：
  - 刚体质量与转动惯量对角项：`MASS`、`IP`(Ixx/Iyy/Izz)
  - 重力：`accgrav/IGRAV,JGRAV,KGRAV`
  - 固定关节：仅 `JOINT` 中 `Fixed` 类型会被映射为 `rigidbody_constraint/fix`
  - 接触关键参数：`BPEN -> default_contact_thickness`，`K -> min_contact_stiffness`，`S_F_C -> pair.friction`
  - 时间积分上限：`INTPAR/HMAX -> settings.simulation.max_time_step_size`
  - 终止时间：优先由同目录 `*_Pos.csv` 末尾时间推断；可用 `--end-time` 覆盖
  - 坐标系处理：OBJ 导出为**刚体局部坐标系**（使用 RM/CM marker 变换），避免重复平移/旋转
- **典型用法**：

  ```powershell
  python assets/tools/rmd_to_scene.py path\to\scene.rmd --out-dir assets/models/basic_contact
  ```

- **显式覆盖终止时间**：

  ```powershell
  python assets/tools/rmd_to_scene.py path\to\scene.rmd --end-time 0.3
  ```

### compare_timeseries.py

- **用途**：比较两个 CSV 时序文件（基准线 vs 模拟结果）并报告最大绝对误差。
- **典型用法**：

  ```powershell
  python assets/tools/compare_timeseries.py baseline.csv sim.csv --max-abs-error 0.02
  ```

### ipc_gap_analysis.py

- **用途**：仅基于已输出 CSV + scene json 做 IPC 越界证据分析（不改求解器）。
- **输出**：
  - `output/figures/{scene}/IPC_min_gap_vs_dhat.svg`
  - `output/figures/{scene}/IPC_min_gap_report.txt`
- **典型用法**：

  ```powershell
  python assets/tools/ipc_gap_analysis.py \
    --scene-json assets/models/basic_contact/basic_contact.json \
    --sim-csv output/basic_contact/Body2_Pos.csv \
    --d-hat 1e-5
  ```

### basic_contact_regression.py

- **用途**：便捷的测试工具，用于运行 RMD 场景回归流程（按需生成场景、运行模拟并与 `Body2_Pos.csv` 基准线对比）。
- **输出位置**：
  - 模拟结果：`output/{rmd文件名称}/Body2_Pos.csv`
  - 对比曲线：`output/figures/{rmd文件名称}/Body2_y_compare.svg`
- **典型用法**：

  ```powershell
  python assets/tools/basic_contact_regression.py \
    --exe build/bin/Release/NexDynDiff.exe \
    --rmd assets/models/basic_contact/basic_contact.rmd
  ```

- **建议**：对齐基准采样频率（如 1000Hz）以避免比较误差被下采样放大：

  ```powershell
  python assets/tools/basic_contact_regression.py \
    --exe build/bin/Release/NexDynDiff.exe \
    --rmd assets/models/basic_contact/basic_contact.rmd \
    --csv-fps 1000
  ```

- **切换 IPC 势能/摩擦模型（通过 scene 接口注入）**：

  ```powershell
  python assets/tools/basic_contact_regression.py \
    --exe build/bin/Release/NexDynDiff.exe \
    --rmd assets/models/basic_contact/basic_contact.rmd \
    --csv-fps 1000 \
    --ipc-barrier-type log \
    --ipc-friction-type c0
  ```

- **启用严格可行性（CCD 安全步长 + 距离阈值检查）**：

  ```powershell
  python assets/tools/basic_contact_regression.py \
    --exe build/bin/Release/NexDynDiff.exe \
    --rmd assets/models/basic_contact/basic_contact.rmd \
    --csv-fps 1000 \
    --strict-feasibility \
    --ccd-eta 0.9
  ```

- **严格阈值模式（超阈值时返回非零退出码）**：

  ```powershell
  python assets/tools/basic_contact_regression.py \
    --exe build/bin/Release/NexDynDiff.exe \
    --rmd assets/models/basic_contact/basic_contact.rmd \
    --max-abs-threshold 0.02 \
    --enforce-threshold
  ```

## C++ 回归测试

如果已构建项目和测试，可通过 CTest（推荐）或直接运行测试可执行文件来运行 BasicContact 回归测试。

**使用 CTest（从 build 目录）**：

```powershell
cd build
ctest -C Release -R BasicContactRegression -V
```

**或直接运行测试可执行文件（Windows 路径）**：

```powershell
build\tests\Release\BasicContactRegression.exe
```

## 备注

- 这些脚本仅作为轻量级辅助工具。如果遇到导入或依赖错误，可创建小型虚拟环境并安装所需包（如有），然后重新运行。
- 如有需要，我可以添加 `requirements.txt` 和简短的 Powershell 包装脚本来自动运行完整回归测试并收集结果。

## 联系方式

如需后续操作，请告诉我：
1. 在此处运行回归测试
2. 添加 `requirements.txt`
3. 创建 CI 步骤以自动运行回归测试

---

## 更新日志

| 日期 | 修改内容 | 修改人 |
|------|----------|--------|
| 2026-02-26 | 初始版本（英文） | - |
| 2026-02-26 | 翻译为中文 | AI Assistant |
