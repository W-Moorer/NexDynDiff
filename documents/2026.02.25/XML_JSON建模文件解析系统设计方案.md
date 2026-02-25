# NexDynDiff XML/JSON 建模文件解析系统设计方案

## 一、当前项目建模流程分析

### 1.1 数据输入方式

当前项目采用 **纯 C++ API 配置方式** 定义仿真场景，主要数据输入方式包括：

| 输入类型 | 文件格式 | 解析方式 | 用途 |
|---------|---------|---------|------|
| 网格数据 | VTK 4.2 | `vtkio` 库 | 四面体/三角形网格几何数据 |
| 网格数据 | OBJ | `tinyobjloader` 库 | 三角形网格几何数据 |
| 场景配置 | **无** | - | 仿真参数、材料属性、边界条件等 |

**关键代码路径**：
- `nexdyndiff/src/utils/mesh_utils.h` - 网格加载接口
- `nexdyndiff/extern/vtkio/src/VTKFile.h` - VTK解析实现
- `examples/main.cpp` - 场景配置示例（硬编码方式）

### 1.2 处理逻辑

```
┌─────────────────────────────────────────────────────────────────┐
│                    当前数据流程                                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   外部文件 (VTK/OBJ)     硬编码配置 (C++ 代码)                   │
│         ↓                        ↓                              │
│   load_vtk/load_obj      Settings + Presets                     │
│         ↓                        ↓                              │
│   Mesh<N> {vertices, conn}   参数初始化                          │
│         ↓                        ↓                              │
│         └───────────→ Simulation ←────────────┘                 │
│                           │                                     │
│                           ↓                                     │
│                    Deformables / RigidBodies / Interactions      │
│                           │                                     │
│                           ↓                                     │
│                    simulation.run()                             │
│                           │                                     │
│                           ↓                                     │
│                    VTK 输出文件                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 1.3 核心数据结构

```cpp
// Settings - 仿真设置
Settings::Output     → 输出目录、帧率、仿真名称
Settings::Simulation → 重力、时间步、自适应步长
Settings::NewtonsMethod → 残差类型、线性求解器、迭代参数
Settings::Execution  → 执行时间、线程数
Settings::Debug      → 调试选项

// 模型参数
Volume::Params       → 体积物体（密度、杨氏模量、泊松比、阻尼、接触参数）
Surface::Params      → 表面物体（密度、杨氏模量、弯曲刚度、应变限制）
Line::Params         → 线状物体（截面半径、杨氏模量）
RigidBody::Params    → 刚体（质量、惯性张量）

// 边界条件
EnergyPrescribedPositions::Params → 固定位置约束
RigidBodyConstraints              → 铰链、滑块、电机、弹簧等约束

// 交互
EnergyFrictionalContact::GlobalParams → 接触厚度、摩擦阈值
ContactParams → 接触参数
EnergyAttachments::Params → 附件约束参数
```

### 1.4 输出要求

```cpp
Settings::Output {
    simulation_name = "scene_name";     // 仿真名称
    output_directory = "path/to/output";// 输出目录
    codegen_directory = "path/to/code"; // 代码生成目录
    fps = 30;                           // 输出帧率
    enable_output = true;               // 是否启用输出
}

// 输出格式: VTK 文件
// 命名规则: {simulation_name}_{mesh_label}_{frame}.vtk
```

---

## 二、系统设计方案

### 2.1 整体架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    XML/JSON 建模文件解析系统架构                          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────────────────┐ │
│  │ XML 文件    │    │ JSON 文件   │    │ 场景描述文件 (.scene)       │ │
│  └──────┬──────┘    └──────┬──────┘    └──────────────┬──────────────┘ │
│         │                  │                          │                │
│         ↓                  ↓                          ↓                │
│  ┌────────────────────────────────────────────────────────────────────┐│
│  │                    SceneFileParser (场景文件解析器)                  ││
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────────┐  ││
│  │  │ XMLParser    │  │ JSONParser   │  │ FormatDetector           │  ││
│  │  │ (tinyxml2)   │  │ (nlohmann)   │  │ (自动检测文件格式)        │  ││
│  │  └──────────────┘  └──────────────┘  └──────────────────────────┘  ││
│  └────────────────────────────────────────────────────────────────────┘│
│                                    │                                   │
│                                    ↓                                   │
│  ┌────────────────────────────────────────────────────────────────────┐│
│  │                    SceneDescription (场景描述数据结构)               ││
│  │  ┌──────────────────────────────────────────────────────────────┐  ││
│  │  │ - SimulationSettings                                          │  ││
│  │  │ - std::vector<DeformableObject>                               │  ││
│  │  │ - std::vector<RigidBody>                                      │  ││
│  │  │ - std::vector<Interaction>                                    │  ││
│  │  │ - std::vector<BoundaryCondition>                              │  ││
│  │  │ - std::vector<ScriptedEvent>                                  │  ││
│  │  └──────────────────────────────────────────────────────────────┘  ││
│  └────────────────────────────────────────────────────────────────────┘│
│                                    │                                   │
│                                    ↓                                   │
│  ┌────────────────────────────────────────────────────────────────────┐│
│  │                    SceneBuilder (场景构建器)                        ││
│  │  - validate()     : 数据验证                                        ││
│  │  - build()        : 构建 Simulation 对象                            ││
│  │  - applyPresets() : 应用预设                                        ││
│  │  - setupOutput()  : 配置输出                                        ││
│  └────────────────────────────────────────────────────────────────────┘│
│                                    │                                   │
│                                    ↓                                   │
│  ┌────────────────────────────────────────────────────────────────────┐│
│  │                    Simulation (现有仿真引擎)                        ││
│  │  simulation.run()                                                   ││
│  └────────────────────────────────────────────────────────────────────┘│
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.2 文件格式设计

#### 2.2.1 JSON 场景文件格式

```json
{
  "version": "1.0",
  "metadata": {
    "name": "hanging_cloth",
    "description": "Simulation of a piece of cloth fixed by two corners",
    "author": "developer",
    "created": "2026-02-25"
  },
  
  "settings": {
    "output": {
      "simulation_name": "hanging_cloth",
      "output_directory": "./output/hanging_cloth",
      "codegen_directory": "./build/compiled",
      "fps": 30,
      "enable_output": true
    },
    "simulation": {
      "gravity": [0.0, 0.0, -9.81],
      "max_time_step_size": 0.01,
      "use_adaptive_time_step": true,
      "init_frictional_contact": false
    },
    "newton": {
      "residual_type": "Acceleration",
      "residual_tolerance": 1.0,
      "linear_system_solver": "CG",
      "max_newton_iterations": 100,
      "project_to_PD": true
    },
    "execution": {
      "end_simulation_time": 5.0,
      "n_threads": -1
    }
  },
  
  "materials": {
    "cotton_fabric": {
      "type": "surface",
      "density": 1000.0,
      "youngs_modulus": 5000.0,
      "poisson_ratio": 0.3,
      "damping": 0.5,
      "bending_stiffness": 0.001,
      "strain_limit": 0.2,
      "elasticity_only": false
    },
    "soft_rubber": {
      "type": "volume",
      "density": 1000.0,
      "youngs_modulus": 50000.0,
      "poisson_ratio": 0.45,
      "damping": 0.1
    },
    "elastic_band": {
      "type": "line",
      "section_radius": 0.005,
      "youngs_modulus": 100000.0,
      "damping": 0.3
    }
  },
  
  "objects": {
    "deformables": [
      {
        "id": "cloth_1",
        "type": "surface",
        "label": "cloth",
        "geometry": {
          "type": "grid",
          "size": [1.0, 1.0],
          "subdivisions": [20, 20]
        },
        "material": "cotton_fabric",
        "transform": {
          "rotation": [0.0, 0.0, 0.0],
          "translation": [0.0, 0.0, 0.0]
        },
        "contact": {
          "enabled": true,
          "thickness": 0.001,
          "friction": 0.5
        }
      },
      {
        "id": "vol_1",
        "type": "volume",
        "label": "rubber_block",
        "geometry": {
          "type": "file",
          "path": "./models/armadillo_tet.vtk"
        },
        "material": "soft_rubber",
        "transform": {
          "translation": [-0.6, 0.0, 0.7]
        }
      },
      {
        "id": "line_1",
        "type": "line",
        "label": "net",
        "geometry": {
          "type": "generated",
          "generator": "triangle_grid",
          "params": {
            "corner_min": [0.0, 0.0],
            "corner_max": [1.0, 1.0],
            "subdivisions": [20, 20]
          }
        },
        "material": "elastic_band"
      }
    ],
    
    "rigidbodies": [
      {
        "id": "floor",
        "label": "floor",
        "geometry": {
          "type": "box",
          "size": [2.0, 2.0, 0.05]
        },
        "mass": 1.0,
        "inertia": "auto",
        "transform": {
          "translation": [0.0, 0.0, -0.025]
        },
        "contact": {
          "thickness": 0.001
        },
        "fixed": true
      },
      {
        "id": "box_1",
        "label": "moving_box",
        "geometry": {
          "type": "box",
          "size": [0.25, 0.25, 0.25]
        },
        "mass": 0.1,
        "transform": {
          "translation": [0.0, 0.0, 0.5]
        }
      }
    ]
  },
  
  "boundary_conditions": [
    {
      "type": "prescribed_position",
      "target": "cloth_1",
      "selector": {
        "type": "inside_aabb",
        "center": [0.5, 0.5, 0.0],
        "halfsize": [0.001, 0.001, 0.001]
      },
      "params": {
        "stiffness": 10000000.0,
        "tolerance": 0.001
      }
    },
    {
      "type": "prescribed_position",
      "target": "cloth_1",
      "selector": {
        "type": "inside_aabb",
        "center": [-0.5, 0.5, 0.0],
        "halfsize": [0.001, 0.001, 0.001]
      }
    },
    {
      "type": "rigidbody_constraint",
      "constraint_type": "fix",
      "target": "floor"
    }
  ],
  
  "interactions": {
    "contact": {
      "global_params": {
        "default_contact_thickness": 0.001,
        "friction_stick_slide_threshold": 0.01,
        "min_contact_stiffness": 100000000.0
      },
      "pairs": [
        {
          "object1": "floor",
          "object2": "vol_1",
          "friction": 1.0
        },
        {
          "object1": "vol_1",
          "object2": "cloth_1",
          "friction": 0.5,
          "enabled": true
        }
      ],
      "disabled_pairs": [
        ["hand", "left_finger"]
      ]
    },
    "attachments": [
      {
        "type": "by_distance",
        "object1": "cloth_2",
        "object2": "cloth_1",
        "selector1": "all",
        "distance_threshold": 0.002,
        "params": {
          "tolerance": 0.01
        }
      }
    ]
  },
  
  "scripted_events": [
    {
      "type": "time_event",
      "start_time": 0.0,
      "end_time": 5.0,
      "target": "cloth_1_bc_left",
      "action": {
        "type": "transform_bc",
        "angular_velocity": 90.0,
        "axis": [1.0, 0.0, 0.0]
      }
    },
    {
      "type": "time_event",
      "start_time": 2.0,
      "end_time": 3.0,
      "action": {
        "type": "set_gravity",
        "gravity": [0.0, 0.0, -10.0],
        "blend": "linear"
      }
    },
    {
      "type": "callback",
      "frequency": "every_timestep",
      "action": {
        "type": "custom_force",
        "script": "magnetic_attraction.lua"
      }
    }
  ]
}
```

#### 2.2.2 XML 场景文件格式

```xml
<?xml version="1.0" encoding="UTF-8"?>
<scene version="1.0">
  <!-- 元数据 -->
  <metadata>
    <name>hanging_cloth</name>
    <description>Simulation of a piece of cloth fixed by two corners</description>
    <author>developer</author>
    <created>2026-02-25</created>
  </metadata>
  
  <!-- 仿真设置 -->
  <settings>
    <output simulation_name="hanging_cloth" 
            output_directory="./output/hanging_cloth"
            codegen_directory="./build/compiled"
            fps="30" 
            enable_output="true"/>
    
    <simulation gravity="0.0 0.0 -9.81"
                max_time_step_size="0.01"
                use_adaptive_time_step="true"
                init_frictional_contact="false"/>
    
    <newton residual_type="Acceleration"
            residual_tolerance="1.0"
            linear_system_solver="CG"
            max_newton_iterations="100"
            project_to_PD="true"/>
    
    <execution end_simulation_time="5.0" n_threads="-1"/>
  </settings>
  
  <!-- 材料定义 -->
  <materials>
    <material id="cotton_fabric" type="surface">
      <density value="1000.0"/>
      <youngs_modulus value="5000.0"/>
      <poisson_ratio value="0.3"/>
      <damping value="0.5"/>
      <bending_stiffness value="0.001"/>
      <strain_limit value="0.2"/>
    </material>
    
    <material id="soft_rubber" type="volume">
      <density value="1000.0"/>
      <youngs_modulus value="50000.0"/>
      <poisson_ratio value="0.45"/>
      <damping value="0.1"/>
    </material>
  </materials>
  
  <!-- 可变形体 -->
  <objects>
    <deformables>
      <deformable id="cloth_1" type="surface" label="cloth" material="cotton_fabric">
        <geometry type="grid">
          <size x="1.0" y="1.0"/>
          <subdivisions nx="20" ny="20"/>
        </geometry>
        <transform translation="0.0 0.0 0.0"/>
        <contact enabled="true" thickness="0.001" friction="0.5"/>
      </deformable>
      
      <deformable id="vol_1" type="volume" label="rubber_block" material="soft_rubber">
        <geometry type="file" path="./models/armadillo_tet.vtk"/>
        <transform translation="-0.6 0.0 0.7"/>
      </deformable>
    </deformables>
    
    <!-- 刚体 -->
    <rigidbodies>
      <rigidbody id="floor" label="floor" mass="1.0" fixed="true">
        <geometry type="box">
          <size x="2.0" y="2.0" z="0.05"/>
        </geometry>
        <transform translation="0.0 0.0 -0.025"/>
        <contact thickness="0.001"/>
      </rigidbody>
    </rigidbodies>
  </objects>
  
  <!-- 边界条件 -->
  <boundary_conditions>
    <prescribed_position target="cloth_1">
      <selector type="inside_aabb" center="0.5 0.5 0.0" halfsize="0.001 0.001 0.001"/>
      <params stiffness="1e7" tolerance="0.001"/>
    </prescribed_position>
    
    <rigidbody_constraint type="fix" target="floor"/>
  </boundary_conditions>
  
  <!-- 交互 -->
  <interactions>
    <contact>
      <global_params default_contact_thickness="0.001"
                     friction_stick_slide_threshold="0.01"
                     min_contact_stiffness="1e8"/>
      
      <friction_pair object1="floor" object2="vol_1" friction="1.0"/>
      <friction_pair object1="vol_1" object2="cloth_1" friction="0.5"/>
      
      <disabled_pair object1="hand" object2="left_finger"/>
    </contact>
  </interactions>
  
  <!-- 脚本事件 -->
  <scripted_events>
    <time_event start_time="0.0" end_time="5.0" target="cloth_1_bc_left">
      <action type="transform_bc" angular_velocity="90.0" axis="1.0 0.0 0.0"/>
    </time_event>
  </scripted_events>
</scene>
```

---

## 三、模块详细设计

### 3.1 文件解析模块

#### 3.1.1 解析器接口设计

```cpp
// File: nexdyndiff/src/scene/SceneParser.h

#pragma once

#include <string>
#include <memory>
#include <filesystem>
#include "SceneDescription.h"

namespace nexdyndiff::scene
{
    // 解析结果状态
    enum class ParseResult {
        Success,
        FileNotFound,
        ParseError,
        ValidationError,
        UnsupportedFormat
    };
    
    // 解析错误信息
    struct ParseError {
        int line;
        int column;
        std::string message;
        std::string context;
    };
    
    // 解析器基类
    class ISceneParser {
    public:
        virtual ~ISceneParser() = default;
        
        // 解析文件
        virtual ParseResult parse(const std::filesystem::path& filepath,
                                  SceneDescription& out_scene,
                                  std::vector<ParseError>& out_errors) = 0;
        
        // 验证场景描述
        virtual bool validate(const SceneDescription& scene,
                             std::vector<ParseError>& out_errors) = 0;
        
        // 支持的文件扩展名
        virtual std::vector<std::string> supported_extensions() const = 0;
    };
    
    // 解析器工厂
    class SceneParserFactory {
    public:
        static std::unique_ptr<ISceneParser> create(const std::string& format);
        static std::unique_ptr<ISceneParser> createFromExtension(const std::filesystem::path& filepath);
        static bool isSupported(const std::filesystem::path& filepath);
    };
}
```

#### 3.1.2 JSON 解析器实现

```cpp
// File: nexdyndiff/src/scene/JSONSceneParser.h

#pragma once

#include "SceneParser.h"
#include <nlohmann/json.hpp>

namespace nexdyndiff::scene
{
    class JSONSceneParser : public ISceneParser {
    public:
        ParseResult parse(const std::filesystem::path& filepath,
                         SceneDescription& out_scene,
                         std::vector<ParseError>& out_errors) override;
        
        bool validate(const SceneDescription& scene,
                     std::vector<ParseError>& out_errors) override;
        
        std::vector<std::string> supported_extensions() const override {
            return {".json", ".scene"};
        }
        
    private:
        // 解析各个模块
        bool parseMetadata(const nlohmann::json& j, SceneDescription& scene, std::vector<ParseError>& errors);
        bool parseSettings(const nlohmann::json& j, SceneDescription& scene, std::vector<ParseError>& errors);
        bool parseMaterials(const nlohmann::json& j, SceneDescription& scene, std::vector<ParseError>& errors);
        bool parseDeformables(const nlohmann::json& j, SceneDescription& scene, std::vector<ParseError>& errors);
        bool parseRigidBodies(const nlohmann::json& j, SceneDescription& scene, std::vector<ParseError>& errors);
        bool parseBoundaryConditions(const nlohmann::json& j, SceneDescription& scene, std::vector<ParseError>& errors);
        bool parseInteractions(const nlohmann::json& j, SceneDescription& scene, std::vector<ParseError>& errors);
        bool parseScriptedEvents(const nlohmann::json& j, SceneDescription& scene, std::vector<ParseError>& errors);
        
        // 辅助方法
        Eigen::Vector3d parseVector3d(const nlohmann::json& j, const std::string& key);
        std::array<double, 3> parseArray3d(const nlohmann::json& arr);
        Transform parseTransform(const nlohmann::json& j);
    };
}
```

```cpp
// File: nexdyndiff/src/scene/JSONSceneParser.cpp

#include "JSONSceneParser.h"
#include <fstream>
#include <sstream>

namespace nexdyndiff::scene
{
    ParseResult JSONSceneParser::parse(const std::filesystem::path& filepath,
                                       SceneDescription& out_scene,
                                       std::vector<ParseError>& out_errors)
    {
        // 检查文件是否存在
        if (!std::filesystem::exists(filepath)) {
            out_errors.push_back({0, 0, "File not found: " + filepath.string(), ""});
            return ParseResult::FileNotFound;
        }
        
        // 读取文件内容
        std::ifstream file(filepath);
        if (!file.is_open()) {
            out_errors.push_back({0, 0, "Cannot open file: " + filepath.string(), ""});
            return ParseResult::ParseError;
        }
        
        // 解析 JSON
        nlohmann::json j;
        try {
            file >> j;
        } catch (const nlohmann::json::parse_error& e) {
            out_errors.push_back({
                static_cast<int>(e.byte),
                0,
                std::string("JSON parse error: ") + e.what(),
                ""
            });
            return ParseResult::ParseError;
        }
        
        // 解析各个模块
        bool success = true;
        success &= parseMetadata(j, out_scene, out_errors);
        success &= parseSettings(j, out_scene, out_errors);
        success &= parseMaterials(j, out_scene, out_errors);
        success &= parseDeformables(j, out_scene, out_errors);
        success &= parseRigidBodies(j, out_scene, out_errors);
        success &= parseBoundaryConditions(j, out_scene, out_errors);
        success &= parseInteractions(j, out_scene, out_errors);
        success &= parseScriptedEvents(j, out_scene, out_errors);
        
        // 设置基础路径（用于解析相对路径）
        out_scene.base_path = std::filesystem::absolute(filepath).parent_path();
        
        return success ? ParseResult::Success : ParseResult::ParseError;
    }
    
    bool JSONSceneParser::parseSettings(const nlohmann::json& j, 
                                        SceneDescription& scene, 
                                        std::vector<ParseError>& errors)
    {
        if (!j.contains("settings")) return true;
        
        const auto& s = j["settings"];
        
        // Output settings
        if (s.contains("output")) {
            const auto& o = s["output"];
            if (o.contains("simulation_name")) scene.settings.output.simulation_name = o["simulation_name"];
            if (o.contains("output_directory")) scene.settings.output.output_directory = o["output_directory"];
            if (o.contains("codegen_directory")) scene.settings.output.codegen_directory = o["codegen_directory"];
            if (o.contains("fps")) scene.settings.output.fps = o["fps"];
            if (o.contains("enable_output")) scene.settings.output.enable_output = o["enable_output"];
        }
        
        // Simulation settings
        if (s.contains("simulation")) {
            const auto& sim = s["simulation"];
            if (sim.contains("gravity")) {
                auto g = sim["gravity"];
                scene.settings.simulation.gravity = Eigen::Vector3d(g[0], g[1], g[2]);
            }
            if (sim.contains("max_time_step_size")) scene.settings.simulation.max_time_step_size = sim["max_time_step_size"];
            if (sim.contains("use_adaptive_time_step")) scene.settings.simulation.use_adaptive_time_step = sim["use_adaptive_time_step"];
            if (sim.contains("init_frictional_contact")) scene.settings.simulation.init_frictional_contact = sim["init_frictional_contact"];
        }
        
        // Newton settings
        if (s.contains("newton")) {
            const auto& n = s["newton"];
            if (n.contains("residual_type")) {
                std::string type = n["residual_type"];
                scene.settings.newton.residual.type = (type == "Force") ? 
                    ResidualType::Force : ResidualType::Acceleration;
            }
            if (n.contains("residual_tolerance")) scene.settings.newton.residual.tolerance = n["residual_tolerance"];
            if (n.contains("max_newton_iterations")) scene.settings.newton.max_newton_iterations = n["max_newton_iterations"];
        }
        
        // Execution settings
        if (s.contains("execution")) {
            const auto& e = s["execution"];
            if (e.contains("end_simulation_time")) scene.settings.execution.end_simulation_time = e["end_simulation_time"];
            if (e.contains("end_frame")) scene.settings.execution.end_frame = e["end_frame"];
            if (e.contains("n_threads")) scene.settings.execution.n_threads = e["n_threads"];
        }
        
        return true;
    }
    
    // ... 其他解析方法的实现
}
```

#### 3.1.3 XML 解析器实现

```cpp
// File: nexdyndiff/src/scene/XMLSceneParser.h

#pragma once

#include "SceneParser.h"

// 前向声明 tinyxml2
namespace tinyxml2 {
    class XMLDocument;
    class XMLElement;
}

namespace nexdyndiff::scene
{
    class XMLSceneParser : public ISceneParser {
    public:
        ParseResult parse(const std::filesystem::path& filepath,
                         SceneDescription& out_scene,
                         std::vector<ParseError>& out_errors) override;
        
        bool validate(const SceneDescription& scene,
                     std::vector<ParseError>& out_errors) override;
        
        std::vector<std::string> supported_extensions() const override {
            return {".xml"};
        }
        
    private:
        bool parseMetadata(tinyxml2::XMLElement* root, SceneDescription& scene, std::vector<ParseError>& errors);
        bool parseSettings(tinyxml2::XMLElement* root, SceneDescription& scene, std::vector<ParseError>& errors);
        bool parseMaterials(tinyxml2::XMLElement* root, SceneDescription& scene, std::vector<ParseError>& errors);
        bool parseObjects(tinyxml2::XMLElement* root, SceneDescription& scene, std::vector<ParseError>& errors);
        bool parseBoundaryConditions(tinyxml2::XMLElement* root, SceneDescription& scene, std::vector<ParseError>& errors);
        bool parseInteractions(tinyxml2::XMLElement* root, SceneDescription& scene, std::vector<ParseError>& errors);
        bool parseScriptedEvents(tinyxml2::XMLElement* root, SceneDescription& scene, std::vector<ParseError>& errors);
        
        // 辅助方法
        Eigen::Vector3d parseVector3dAttr(tinyxml2::XMLElement* elem, const char* attrName);
        double parseDoubleAttr(tinyxml2::XMLElement* elem, const char* attrName, double defaultVal = 0.0);
        int parseIntAttr(tinyxml2::XMLElement* elem, const char* attrName, int defaultVal = 0);
        std::string parseStringAttr(tinyxml2::XMLElement* elem, const char* attrName);
    };
}
```

### 3.2 场景描述数据结构

```cpp
// File: nexdyndiff/src/scene/SceneDescription.h

#pragma once

#include <string>
#include <vector>
#include <optional>
#include <map>
#include <filesystem>
#include <Eigen/Dense>
#include "Settings.h"

namespace nexdyndiff::scene
{
    // ==================== 变换 ====================
    struct Transform {
        Eigen::Vector3d translation = Eigen::Vector3d::Zero();
        Eigen::Vector3d rotation_euler = Eigen::Vector3d::Zero(); // 欧拉角（度）
        double rotation_angle = 0.0;
        Eigen::Vector3d rotation_axis = Eigen::Vector3d::UnitZ();
        std::optional<Eigen::Matrix3d> rotation_matrix;
    };
    
    // ==================== 材料定义 ====================
    struct MaterialParams {
        std::string id;
        std::string type; // "volume", "surface", "line"
        
        // 通用参数
        double density = 1000.0;
        double youngs_modulus = 10000.0;
        double poisson_ratio = 0.3;
        double damping = 0.1;
        
        // 表面/体积特有参数
        double strain_limit = 0.0;
        double strain_limit_stiffness = 100.0;
        bool elasticity_only = false;
        
        // 表面特有参数
        double bending_stiffness = 0.001;
        bool flat_rest_angle = true;
        
        // 线特有参数
        double section_radius = 0.005;
    };
    
    // ==================== 几何定义 ====================
    struct GeometryDefinition {
        enum class Type {
            File,           // 从文件加载
            Grid,           // 网格生成
            Box,            // 立方体
            Sphere,         // 球体
            Cylinder,       // 圆柱体
            Generated       // 程序化生成
        };
        
        Type type = Type::File;
        
        // 文件路径（Type::File）
        std::string file_path;
        
        // 网格参数（Type::Grid）
        std::array<double, 2> size_2d = {1.0, 1.0};
        std::array<double, 3> size_3d = {1.0, 1.0, 1.0};
        std::array<int, 2> subdivisions_2d = {10, 10};
        std::array<int, 3> subdivisions_3d = {5, 5, 5};
        
        // 生成器名称（Type::Generated）
        std::string generator_name;
        std::map<std::string, double> generator_params;
        
        // 球体参数
        double sphere_radius = 1.0;
        int sphere_subdivisions = 3;
    };
    
    // ==================== 接触参数 ====================
    struct ContactParamsDef {
        bool enabled = true;
        double thickness = 0.001;
        double friction = 0.5;
    };
    
    // ==================== 可变形体 ====================
    struct DeformableObject {
        std::string id;
        std::string label;
        std::string type; // "volume", "surface", "line"
        
        GeometryDefinition geometry;
        std::string material_id;  // 引用材料定义
        Transform transform;
        ContactParamsDef contact;
        
        // 直接参数（可选，覆盖材料定义）
        std::optional<MaterialParams> inline_material;
    };
    
    // ==================== 刚体 ====================
    struct RigidBody {
        std::string id;
        std::string label;
        
        GeometryDefinition geometry;
        double mass = 1.0;
        std::string inertia_type = "auto"; // "auto", "diagonal", "full"
        std::optional<Eigen::Vector3d> inertia_diagonal;
        std::optional<Eigen::Matrix3d> inertia_tensor;
        
        Transform transform;
        ContactParamsDef contact;
        
        bool fixed = false;
    };
    
    // ==================== 边界条件 ====================
    struct SelectorDefinition {
        enum class Type {
            All,            // 所有点
            InsideAABB,     // AABB 内部
            OutsideAABB,    // AABB 外部
            Indices,        // 指定索引
            ByDistance      // 按距离
        };
        
        Type type = Type::All;
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        Eigen::Vector3d halfsize = Eigen::Vector3d::Ones();
        std::vector<int> indices;
        double distance_threshold = 0.0;
    };
    
    struct BoundaryCondition {
        enum class Type {
            PrescribedPosition,
            RigidBodyConstraint
        };
        
        Type type;
        std::string target_id;  // 目标对象 ID
        
        // PrescribedPosition
        SelectorDefinition selector;
        double stiffness = 1e7;
        double tolerance = 1e-3;
        
        // RigidBodyConstraint
        std::string constraint_type; // "fix", "hinge", "slider", "prismatic", "motor", "spring", ...
        
        // 约束参数
        Eigen::Vector3d constraint_point = Eigen::Vector3d::Zero();
        Eigen::Vector3d constraint_axis = Eigen::Vector3d::UnitZ();
        double constraint_value = 0.0;
        std::string target_id2;  // 第二目标（用于双刚体约束）
    };
    
    // ==================== 交互 ====================
    struct ContactPair {
        std::string object1_id;
        std::string object2_id;
        double friction = 0.5;
        bool enabled = true;
    };
    
    struct AttachmentDefinition {
        enum class Type { ByDistance, ByIndices };
        Type type = Type::ByDistance;
        
        std::string object1_id;
        std::string object2_id;
        SelectorDefinition selector1;
        SelectorDefinition selector2;
        
        double distance_threshold = 0.01;
        double tolerance = 0.01;
    };
    
    struct InteractionsDefinition {
        // 全局接触参数
        double default_contact_thickness = 0.001;
        double friction_stick_slide_threshold = 0.01;
        double min_contact_stiffness = 1e8;
        
        // 接触对
        std::vector<ContactPair> contact_pairs;
        std::vector<std::pair<std::string, std::string>> disabled_pairs;
        
        // 附件
        std::vector<AttachmentDefinition> attachments;
    };
    
    // ==================== 脚本事件 ====================
    struct ScriptedEvent {
        enum class Type { TimeEvent, Callback };
        Type type = Type::TimeEvent;
        
        // 时间事件
        double start_time = 0.0;
        double end_time = 0.0;
        
        // 目标（边界条件 ID 或特殊标识）
        std::string target_id;
        
        // 动作类型
        std::string action_type; // "transform_bc", "set_gravity", "set_friction", "custom_force"
        
        // 动作参数
        Eigen::Vector3d translation = Eigen::Vector3d::Zero();
        double angular_velocity = 0.0;
        Eigen::Vector3d rotation_axis = Eigen::Vector3d::UnitZ();
        Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
        std::string blend_type = "linear";
        double friction_value = 0.0;
        
        // 回调脚本
        std::string script_path;
    };
    
    // ==================== 场景描述 ====================
    struct SceneDescription {
        // 元数据
        std::string name;
        std::string description;
        std::string author;
        std::string version = "1.0";
        std::filesystem::path base_path;  // 场景文件所在目录
        
        // 设置
        Settings settings;
        
        // 材料库
        std::map<std::string, MaterialParams> materials;
        
        // 对象
        std::vector<DeformableObject> deformables;
        std::vector<RigidBody> rigidbodies;
        
        // 边界条件
        std::vector<BoundaryCondition> boundary_conditions;
        
        // 交互
        InteractionsDefinition interactions;
        
        // 脚本事件
        std::vector<ScriptedEvent> scripted_events;
    };
}
```

### 3.3 场景构建器

```cpp
// File: nexdyndiff/src/scene/SceneBuilder.h

#pragma once

#include "SceneDescription.h"
#include "Simulation.h"
#include <functional>

namespace nexdyndiff::scene
{
    // 构建结果
    struct BuildResult {
        bool success = false;
        std::vector<std::string> errors;
        std::vector<std::string> warnings;
    };
    
    // 验证级别
    enum class ValidationLevel {
        Strict,     // 严格验证，任何问题都报错
        Normal,     // 正常验证，允许部分警告
        Relaxed     // 宽松验证，仅报告致命错误
    };
    
    class SceneBuilder {
    public:
        explicit SceneBuilder(const SceneDescription& description);
        
        // 验证场景描述
        BuildResult validate(ValidationLevel level = ValidationLevel::Normal);
        
        // 构建仿真对象
        BuildResult build(Simulation& out_simulation);
        
        // 设置回调
        void setProgressCallback(std::function<void(const std::string& stage, double progress)> callback);
        
    private:
        const SceneDescription& m_description;
        std::function<void(const std::string&, double)> m_progressCallback;
        
        // 构建阶段的内部状态
        struct BuildContext {
            std::map<std::string, PointSetHandler> point_sets;
            std::map<std::string, Volume::Handler> volumes;
            std::map<std::string, Surface::Handler> surfaces;
            std::map<std::string, Line::Handler> lines;
            std::map<std::string, RigidBodyHandler> rigidbodies;
            std::map<std::string, ContactHandler> contacts;
            std::map<std::string, EnergyPrescribedPositions::Handler> prescribed_positions;
        };
        
        // 构建阶段
        bool buildSettings(Simulation& sim, BuildContext& ctx, BuildResult& result);
        bool buildMaterials(BuildContext& ctx, BuildResult& result);
        bool buildDeformables(Simulation& sim, BuildContext& ctx, BuildResult& result);
        bool buildRigidBodies(Simulation& sim, BuildContext& ctx, BuildResult& result);
        bool buildBoundaryConditions(Simulation& sim, BuildContext& ctx, BuildResult& result);
        bool buildInteractions(Simulation& sim, BuildContext& ctx, BuildResult& result);
        bool buildScriptedEvents(Simulation& sim, BuildContext& ctx, BuildResult& result);
        bool buildOutput(Simulation& sim, BuildContext& ctx, BuildResult& result);
        
        // 辅助方法
        MaterialParams resolveMaterial(const std::string& material_id, const DeformableObject& obj);
        Mesh<4> loadTetMesh(const GeometryDefinition& geom);
        Mesh<3> loadTriMesh(const GeometryDefinition& geom);
        Mesh<2> loadLineMesh(const GeometryDefinition& geom);
        void applyTransform(PointSetHandler& handler, const Transform& transform);
        void applyTransform(RigidBodyHandler& handler, const Transform& transform);
        
        // 验证方法
        bool validateGeometry(const GeometryDefinition& geom, BuildResult& result);
        bool validateMaterial(const MaterialParams& mat, BuildResult& result);
        bool validateReferences(const SceneDescription& desc, BuildResult& result);
    };
}
```

```cpp
// File: nexdyndiff/src/scene/SceneBuilder.cpp

#include "SceneBuilder.h"
#include "mesh_utils.h"
#include "mesh_generators.h"
#include <filesystem>

namespace nexdyndiff::scene
{
    SceneBuilder::SceneBuilder(const SceneDescription& description)
        : m_description(description)
    {}
    
    BuildResult SceneBuilder::validate(ValidationLevel level)
    {
        BuildResult result;
        result.success = true;
        
        // 1. 验证设置
        if (m_description.settings.output.simulation_name.empty()) {
            result.errors.push_back("Simulation name is required");
            result.success = false;
        }
        
        // 2. 验证材料引用
        for (const auto& obj : m_description.deformables) {
            if (!obj.inline_material.has_value() && 
                m_description.materials.find(obj.material_id) == m_description.materials.end()) {
                result.errors.push_back("Unknown material: " + obj.material_id + " (object: " + obj.id + ")");
                result.success = false;
            }
        }
        
        // 3. 验证几何定义
        for (const auto& obj : m_description.deformables) {
            if (!validateGeometry(obj.geometry, result)) {
                result.success = false;
            }
        }
        
        // 4. 验证边界条件引用
        for (const auto& bc : m_description.boundary_conditions) {
            bool found = false;
            for (const auto& d : m_description.deformables) {
                if (d.id == bc.target_id) { found = true; break; }
            }
            for (const auto& r : m_description.rigidbodies) {
                if (r.id == bc.target_id) { found = true; break; }
            }
            if (!found) {
                result.errors.push_back("Boundary condition targets unknown object: " + bc.target_id);
                result.success = false;
            }
        }
        
        // 5. 验证交互引用
        for (const auto& pair : m_description.interactions.contact_pairs) {
            // ... 验证对象引用
        }
        
        return result;
    }
    
    BuildResult SceneBuilder::build(Simulation& out_simulation)
    {
        BuildResult result;
        BuildContext ctx;
        
        // 阶段 1: 应用设置
        if (!buildSettings(out_simulation, ctx, result)) {
            return result;
        }
        
        // 阶段 2: 构建可变形体
        if (!buildDeformables(out_simulation, ctx, result)) {
            return result;
        }
        
        // 阶段 3: 构建刚体
        if (!buildRigidBodies(out_simulation, ctx, result)) {
            return result;
        }
        
        // 阶段 4: 设置边界条件
        if (!buildBoundaryConditions(out_simulation, ctx, result)) {
            return result;
        }
        
        // 阶段 5: 设置交互
        if (!buildInteractions(out_simulation, ctx, result)) {
            return result;
        }
        
        // 阶段 6: 设置脚本事件
        if (!buildScriptedEvents(out_simulation, ctx, result)) {
            return result;
        }
        
        result.success = true;
        return result;
    }
    
    bool SceneBuilder::buildDeformables(Simulation& sim, BuildContext& ctx, BuildResult& result)
    {
        for (const auto& obj : m_description.deformables) {
            m_progressCallback("Building deformable: " + obj.id, 0.0);
            
            MaterialParams mat = resolveMaterial(obj.material_id, obj);
            
            if (obj.type == "volume") {
                // 加载或生成四面体网格
                auto mesh = loadTetMesh(obj.geometry);
                
                // 构建参数
                Volume::Params params;
                params.inertia.density = mat.density;
                params.inertia.damping = mat.damping;
                params.strain.youngs_modulus = mat.youngs_modulus;
                params.strain.poisson_ratio = mat.poisson_ratio;
                params.strain.elasticity_only = mat.elasticity_only;
                
                // 添加到仿真
                auto handler = sim.presets->deformables->add_volume(
                    obj.label, mesh.vertices, mesh.conn, params);
                
                // 应用变换
                applyTransform(handler.point_set, obj.transform);
                
                // 存储
                ctx.volumes[obj.id] = handler;
                ctx.point_sets[obj.id] = handler.point_set;
                ctx.contacts[obj.id] = handler.contact;
                
            } else if (obj.type == "surface") {
                auto mesh = loadTriMesh(obj.geometry);
                
                Surface::Params params;
                params.inertia.density = mat.density;
                params.inertia.damping = mat.damping;
                params.strain.youngs_modulus = mat.youngs_modulus;
                params.bending.stiffness = mat.bending_stiffness;
                
                auto handler = sim.presets->deformables->add_surface(
                    obj.label, mesh.vertices, mesh.conn, params);
                
                applyTransform(handler.point_set, obj.transform);
                
                ctx.surfaces[obj.id] = handler;
                ctx.point_sets[obj.id] = handler.point_set;
                ctx.contacts[obj.id] = handler.contact;
                
            } else if (obj.type == "line") {
                auto mesh = loadLineMesh(obj.geometry);
                
                Line::Params params;
                params.section_radius = mat.section_radius;
                params.youngs_modulus = mat.youngs_modulus;
                params.damping = mat.damping;
                
                auto handler = sim.presets->deformables->add_line(
                    obj.label, mesh.vertices, mesh.conn, params);
                
                applyTransform(handler.point_set, obj.transform);
                
                ctx.lines[obj.id] = handler;
                ctx.point_sets[obj.id] = handler.point_set;
                ctx.contacts[obj.id] = handler.contact;
            }
        }
        
        return true;
    }
    
    bool SceneBuilder::buildBoundaryConditions(Simulation& sim, BuildContext& ctx, BuildResult& result)
    {
        for (const auto& bc : m_description.boundary_conditions) {
            if (bc.type == BoundaryCondition::Type::PrescribedPosition) {
                // 查找目标
                auto it = ctx.point_sets.find(bc.target_id);
                if (it == ctx.point_sets.end()) {
                    result.errors.push_back("Unknown target for boundary condition: " + bc.target_id);
                    return false;
                }
                
                PointSetHandler target = it->second;
                
                // 应用选择器
                EnergyPrescribedPositions::Params params;
                params.stiffness = bc.stiffness;
                params.tolerance = bc.tolerance;
                
                if (bc.selector.type == SelectorDefinition::Type::InsideAABB) {
                    auto handler = sim.deformables->prescribed_positions->add_inside_aabb(
                        target, bc.selector.center, bc.selector.halfsize, params);
                    ctx.prescribed_positions[bc.target_id + "_bc"] = handler;
                    
                } else if (bc.selector.type == SelectorDefinition::Type::OutsideAABB) {
                    auto handler = sim.deformables->prescribed_positions->add_outside_aabb(
                        target, bc.selector.center, bc.selector.halfsize, params);
                    ctx.prescribed_positions[bc.target_id + "_bc"] = handler;
                }
                
            } else if (bc.type == BoundaryCondition::Type::RigidBodyConstraint) {
                auto it = ctx.rigidbodies.find(bc.target_id);
                if (it == ctx.rigidbodies.end()) {
                    result.errors.push_back("Unknown rigid body for constraint: " + bc.target_id);
                    return false;
                }
                
                RigidBodyHandler rb = it->second;
                
                if (bc.constraint_type == "fix") {
                    sim.rigidbodies->add_constraint_fix(rb);
                } else if (bc.constraint_type == "hinge") {
                    sim.rigidbodies->add_constraint_hinge(rb, bc.constraint_point, bc.constraint_axis);
                }
                // ... 其他约束类型
            }
        }
        
        return true;
    }
    
    // ... 其他构建方法的实现
}
```

### 3.4 错误处理与数据验证

```cpp
// File: nexdyndiff/src/scene/SceneValidator.h

#pragma once

#include "SceneDescription.h"
#include <regex>

namespace nexdyndiff::scene
{
    // 验证规则
    struct ValidationRule {
        std::string field_path;
        std::string rule_type;  // "required", "range", "regex", "custom"
        std::string rule_value;
        std::string error_message;
    };
    
    // 预定义验证规则
    const std::vector<ValidationRule> g_validationRules = {
        {"settings.output.simulation_name", "required", "", "Simulation name is required"},
        {"settings.simulation.gravity", "range", "-1000,1000", "Gravity value out of range"},
        {"materials.*.youngs_modulus", "range", "0,1e12", "Young's modulus must be positive"},
        {"materials.*.density", "range", "0,100000", "Density must be positive"},
        {"objects.deformables.*.id", "regex", "^[a-zA-Z_][a-zA-Z0-9_]*$", "Invalid object ID format"},
    };
    
    class SceneValidator {
    public:
        // 验证场景描述
        static bool validate(const SceneDescription& scene,
                            std::vector<ParseError>& out_errors,
                            ValidationLevel level = ValidationLevel::Normal);
        
        // 验证单个值
        static bool validateValue(const std::string& path, 
                                 const std::string& value,
                                 const ValidationRule& rule);
        
        // 语义验证（引用完整性、逻辑一致性）
        static bool validateSemantics(const SceneDescription& scene,
                                     std::vector<ParseError>& out_errors);
        
        // 验证文件路径
        static bool validateFilePath(const std::filesystem::path& path,
                                    const std::filesystem::path& basePath,
                                    std::vector<ParseError>& out_errors);
        
    private:
        // 检查引用完整性
        static bool checkReferences(const SceneDescription& scene,
                                   std::vector<ParseError>& out_errors);
        
        // 检查几何有效性
        static bool checkGeometry(const GeometryDefinition& geom,
                                 std::vector<ParseError>& out_errors);
        
        // 检查参数范围
        static bool checkParameterRanges(const SceneDescription& scene,
                                        std::vector<ParseError>& out_errors);
    };
}
```

### 3.5 性能优化考虑

```cpp
// File: nexdyndiff/src/scene/SceneCache.h

#pragma once

#include "SceneDescription.h"
#include <unordered_map>
#include <mutex>
#include <chrono>

namespace nexdyndiff::scene
{
    // 缓存条目
    struct CacheEntry {
        SceneDescription scene;
        std::filesystem::file_time_type last_modified;
        std::chrono::steady_clock::time_point last_accessed;
        size_t access_count = 0;
    };
    
    // 场景缓存
    class SceneCache {
    public:
        static SceneCache& instance() {
            static SceneCache cache;
            return cache;
        }
        
        // 获取缓存的场景
        std::optional<SceneDescription> get(const std::filesystem::path& filepath) {
            std::lock_guard<std::mutex> lock(m_mutex);
            
            auto it = m_cache.find(filepath.string());
            if (it == m_cache.end()) {
                return std::nullopt;
            }
            
            // 检查文件是否修改
            auto current_modified = std::filesystem::last_write_time(filepath);
            if (current_modified != it->second.last_modified) {
                m_cache.erase(it);
                return std::nullopt;
            }
            
            // 更新访问时间
            it->second.last_accessed = std::chrono::steady_clock::now();
            it->second.access_count++;
            
            return it->second.scene;
        }
        
        // 存储场景到缓存
        void put(const std::filesystem::path& filepath, const SceneDescription& scene) {
            std::lock_guard<std::mutex> lock(m_mutex);
            
            CacheEntry entry;
            entry.scene = scene;
            entry.last_modified = std::filesystem::last_write_time(filepath);
            entry.last_accessed = std::chrono::steady_clock::now();
            
            m_cache[filepath.string()] = entry;
            
            // 检查缓存大小，必要时清理
            if (m_cache.size() > m_maxSize) {
                evictLRU();
            }
        }
        
        // 清除缓存
        void clear() {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_cache.clear();
        }
        
        // 设置最大缓存大小
        void setMaxSize(size_t size) { m_maxSize = size; }
        
    private:
        SceneCache() = default;
        
        void evictLRU() {
            // 移除最近最少使用的条目
            auto oldest = std::min_element(m_cache.begin(), m_cache.end(),
                [](const auto& a, const auto& b) {
                    return a.second.last_accessed < b.second.last_accessed;
                });
            
            if (oldest != m_cache.end()) {
                m_cache.erase(oldest);
            }
        }
        
        std::unordered_map<std::string, CacheEntry> m_cache;
        std::mutex m_mutex;
        size_t m_maxSize = 50;
    };
}
```

```cpp
// File: nexdyndiff/src/scene/SceneLoader.h

#pragma once

#include "SceneParser.h"
#include "SceneBuilder.h"
#include "SceneCache.h"
#include "Simulation.h"
#include <functional>

namespace nexdyndiff::scene
{
    // 加载选项
    struct LoadOptions {
        bool use_cache = true;              // 使用缓存
        bool validate_strict = false;       // 严格验证
        bool parallel_geometry = true;      // 并行加载几何
        std::filesystem::path base_path;    // 基础路径（覆盖文件路径）
    };
    
    // 加载状态
    struct LoadStatus {
        enum class State {
            Idle,
            Parsing,
            Validating,
            BuildingGeometry,
            BuildingScene,
            Complete,
            Error
        };
        
        State state = State::Idle;
        double progress = 0.0;
        std::string current_stage;
        std::vector<ParseError> errors;
    };
    
    // 场景加载器（高级接口）
    class SceneLoader {
    public:
        // 从文件加载场景
        static std::unique_ptr<Simulation> load(
            const std::filesystem::path& filepath,
            const LoadOptions& options = LoadOptions(),
            LoadStatus* status = nullptr);
        
        // 异步加载
        static void loadAsync(
            const std::filesystem::path& filepath,
            std::function<void(std::unique_ptr<Simulation>)> onComplete,
            std::function<void(const LoadStatus&)> onProgress = nullptr,
            const LoadOptions& options = LoadOptions());
        
        // 仅解析场景描述（不构建）
        static std::optional<SceneDescription> parseOnly(
            const std::filesystem::path& filepath,
            std::vector<ParseError>& out_errors);
        
        // 重新加载场景（热重载）
        static bool reload(Simulation& simulation,
                          const std::filesystem::path& filepath,
                          const LoadOptions& options = LoadOptions());
    };
}
```

---

## 四、目录结构

```
nexdyndiff/src/scene/
├── include.h                    # 公共头文件
├── SceneParser.h               # 解析器接口
├── SceneParser.cpp
├── JSONSceneParser.h           # JSON 解析器
├── JSONSceneParser.cpp
├── XMLSceneParser.h            # XML 解析器
├── XMLSceneParser.cpp
├── SceneDescription.h          # 场景描述数据结构
├── SceneBuilder.h              # 场景构建器
├── SceneBuilder.cpp
├── SceneValidator.h            # 验证器
├── SceneValidator.cpp
├── SceneCache.h                # 缓存系统
├── SceneLoader.h               # 高级加载接口
├── SceneLoader.cpp
└── predefined/
    ├── PredefinedMaterials.h   # 预定义材料
    └── PredefinedMaterials.cpp
```

---

## 五、使用示例

### 5.1 基本使用

```cpp
#include <nexdyndiff>
#include <nexdyndiff/scene/SceneLoader.h>

int main()
{
    // 方式 1: 直接加载场景文件
    auto simulation = nexdyndiff::scene::SceneLoader::load("scenes/hanging_cloth.json");
    if (simulation) {
        simulation->run();
    }
    
    // 方式 2: 带选项和状态跟踪
    nexdyndiff::scene::LoadOptions options;
    options.validate_strict = true;
    options.use_cache = true;
    
    nexdyndiff::scene::LoadStatus status;
    auto sim = nexdyndiff::scene::SceneLoader::load("scenes/complex_scene.json", options, &status);
    
    if (status.state == nexdyndiff::scene::LoadStatus::State::Error) {
        for (const auto& err : status.errors) {
            std::cerr << "Error at line " << err.line << ": " << err.message << std::endl;
        }
        return 1;
    }
    
    sim->run();
    
    return 0;
}
```

### 5.2 异步加载

```cpp
#include <nexdyndiff/scene/SceneLoader.h>

void loadSceneAsync()
{
    nexdyndiff::scene::SceneLoader::loadAsync(
        "scenes/large_scene.json",
        [](std::unique_ptr<nexdyndiff::Simulation> sim) {
            // 加载完成回调
            if (sim) {
                sim->run();
            }
        },
        [](const nexdyndiff::scene::LoadStatus& status) {
            // 进度回调
            std::cout << "Loading: " << status.current_stage 
                      << " (" << (status.progress * 100) << "%)" << std::endl;
        }
    );
}
```

### 5.3 热重载

```cpp
#include <nexdyndiff/scene/SceneLoader.h>

// 在开发模式下支持场景文件热重载
void hotReloadExample()
{
    auto simulation = nexdyndiff::scene::SceneLoader::load("scenes/dev_scene.json");
    
    // ... 运行仿真 ...
    
    // 当场景文件被修改后，重新加载
    if (fileChanged("scenes/dev_scene.json")) {
        nexdyndiff::scene::SceneLoader::reload(*simulation, "scenes/dev_scene.json");
    }
}
```

---

## 六、错误处理策略

### 6.1 错误分类

| 错误级别 | 类型 | 处理方式 |
|---------|------|---------|
| 致命错误 | 文件不存在、解析失败、引用缺失 | 终止加载，返回错误信息 |
| 错误 | 参数范围无效、格式错误 | 记录错误，终止加载 |
| 警告 | 未使用的定义、过时的参数 | 记录警告，继续加载 |
| 信息 | 加载进度、优化提示 | 输出日志 |

### 6.2 错误信息格式

```json
{
  "error_code": "SCENE_PARSE_001",
  "severity": "error",
  "location": {
    "file": "scenes/hanging_cloth.json",
    "line": 42,
    "column": 15,
    "path": "objects.deformables[0].material"
  },
  "message": "Unknown material reference: 'unknown_material'",
  "suggestion": "Did you mean 'cotton_fabric'?"
}
```

---

## 七、扩展性设计

### 7.1 自定义解析器

```cpp
// 注册自定义解析器
class CustomSceneParser : public nexdyndiff::scene::ISceneParser {
    // ... 实现
};

nexdyndiff::scene::SceneParserFactory::registerParser(".custom", 
    []() { return std::make_unique<CustomSceneParser>(); });
```

### 7.2 自定义材料类型

```cpp
// 在场景文件中定义新材料类型
{
  "materials": {
    "custom_material": {
      "type": "custom",
      "plugin": "plugins/custom_material.so",
      "params": {
        "custom_param1": 100.0,
        "custom_param2": "value"
      }
    }
  }
}
```

### 7.3 自定义几何生成器

```cpp
// 注册自定义几何生成器
nexdyndiff::scene::GeometryFactory::registerGenerator("parametric_surface",
    [](const std::map<std::string, double>& params) -> Mesh<3> {
        // 自定义几何生成逻辑
        return generateCustomMesh(params);
    });
```

---

## 八、总结

本设计方案提供了一个完整的 XML/JSON 建模文件解析系统，具有以下特点：

1. **文件解析模块**：支持 JSON 和 XML 格式，可扩展其他格式
2. **数据转换机制**：将场景描述映射到现有的 `Simulation` API
3. **错误处理**：多级错误分类，详细的错误信息，支持验证级别配置
4. **性能优化**：缓存系统、异步加载、并行几何处理
5. **可扩展性**：支持自定义解析器、材料类型、几何生成器

该系统设计清晰、模块化，易于维护和扩展，可以无缝集成到现有的 NexDynDiff 项目中。
