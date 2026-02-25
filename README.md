# NexDynDiff

NexDynDiff 是一个基于 C++ 的高性能物理仿真平台，专注于刚体与可变形物体的强耦合仿真。该项目是 Stark 项目的分支和延续。

NexDynDiff 延续了 Stark 项目采用强大的符号微分和代码生成引擎的优势，允许以简洁的形式表述非线性动力学问题的全局变分形式。在 NexDynDiff 中添加新模型（如材料、关节、交互等）就像以符号形式指定它们的能量势能一样简单。

## 项目来源

NexDynDiff 源自 Stark 项目，由 RWTH Aachen University 开发并在 ICRA 2024 上发表了相关论文。Stark 的核心创新在于：

- **统一仿真框架**：将刚体动力学、可变形体（FEM）、布料（离散壳）和摩擦接触统一在一个隐式时间积分框架内
- **符号微分引擎**：通过 SymX 符号计算引擎自动生成能量函数的梯度和 Hessian 矩阵
- **IPC 接触处理**：采用 Incremental Potential Contact 方法实现无穿透的鲁棒接触仿真

## 技术特点

| 特性 | 描述 |
|------|------|
| 编程语言 | C++17 |
| 构建系统 | CMake 3.15+ |
| 数值计算 | Eigen 3.x |

## 功能特性

### 模型

NexDynDiff 不仅仅是一个模拟器，它还是用于可变形物体、刚体和摩擦接触的常用势能库：

- **可变形物体**
  - 2D 和 3D 线性有限元方法（三角形和四面体），分别采用 Neo-Hookean 和 Stable Neo-Hookean 本构模型
  - 离散壳模型（用于布料和刚性壳）
  - 应变限制（Strain limiting）
  - 惯性和材料阻尼

- **刚体**
  - 柔性关节（线性）
  - 平滑力/扭矩限制电机
  - 全面的约束集合（球关节、铰链关节、滑块等）

- **摩擦接触**（基于三角形网格）
  - IPC（Incremental Potential Contact）

- **附件**（基于三角形网格距离）

### 求解器

- **符号微分**
  - 自动生成梯度和 Hessian 矩阵
  - 并行自主全局评估和组装

- **基于三角形的碰撞检测**

- **求解器**
  - 牛顿法（Newton's Method）
  - 无交线搜索（Intersection free line search）
  - 共轭梯度（CG）或直接 LU 线性求解器
  - 可选的自适应时间步长
  - 可选的元素 Hessian 数值 PSD 投影

- **基于事件的脚本**

## 构建说明

### 环境要求

- CMake 3.15+
- C++17 编译器
- Windows: Visual Studio 2022 或更高版本
- Linux: GCC 9+ 或 Clang 10+

### 构建步骤

```bash
# 创建构建目录
mkdir build
cd build

# 配置项目
cmake .. -G "Visual Studio 17 2022" -A x64

# 编译
cmake --build . --config Release
```

### 编译器配置

默认情况下，在 Windows 上使用 MSVC 编译器。如需指定其他编译器，可以使用：

```cpp
nexdyndiff::set_compiler_command("\"C:\\Program Files\\Microsoft Visual Studio\\2022\\Community\\VC\\Auxiliary\\Build\\vcvarsx86_amd64.bat\"");
```

## 示例

项目包含多个 C++ 示例，位于 `examples/` 目录下：

- 刚体约束测试场景
- 悬挂网格
- 悬挂布料
- 可变形盒子
- 附件
- 组合材料
- 变形体与刚体碰撞
- 简单抓取
- 扭曲布料
- 磁性可变形体
- 从文件加载网格

## 使用方法

NexDynDiff 代码始终遵循相同的结构：

1. 定义全局设置和参数
2. 添加对象、边界条件并定义交互（可以使用预设或自定义对象）
3. 可选：指定时间相关事件（脚本）
4. 运行

## 相关研究

使用 Stark 的相关研究论文：

- ["Micropolar Elasticity in Physically-Based Animation"](https://www.animation.rwth-aachen.de/publication/0582/) - Löschner et al., 2023
- ["Curved Three-Director Cosserat Shells with Strong Coupling"](https://www.animation.rwth-aachen.de/publication/0589/) - Löschner et al., 2024
- ["Strongly Coupled Simulation of Magnetic Rigid Bodies"](https://www.animation.rwth-aachen.de/publication/0590/) - Westhofen et al., 2024

## 致谢

本项目基于 Stark 项目开发，衷心感谢 Stark 项目的开发者们：

- [José Antonio Fernández-Fernández](https://github.com/JoseAntFer)
- [Fabian Löschner](https://github.com/w1th0utnam3)

同时感谢 Robert Bosch GmbH 对 Stark 项目早期开发（2019-2021）的慷慨资助。

## 原始论文

如果您在研究中使用了本项目或 Stark，请引用以下论文：

```bibtex
@InProceedings{FLL+24,
  author={Fernández-Fernández, José Antonio and Lange, Ralph and Laible, Stefan and Arras, Kai O. and Bender, Jan},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)},
  title={nexdyndiff: A Unified Framework for Strongly Coupled Simulation of Rigid and Deformable Bodies with Frictional Contact},
  year={2024},
  pages={16888-16894},
  doi={10.1109/ICRA57147.2024.10610574}}
}
```

