#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>
#include <atomic>

#include "nexdyndiff.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

using namespace Catch::Matchers;

namespace
{
	std::filesystem::path MakeTempDir()
	{
		const auto timestamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
		const std::filesystem::path dir = std::filesystem::path("temps") / "scene_tests" / std::to_string(timestamp);
		std::filesystem::create_directories(dir);
		return dir;
	}

	void WriteFile(const std::filesystem::path& path, const std::string& content)
	{
		std::ofstream out(path, std::ios::out | std::ios::binary);
		out << content;
	}
}

TEST_CASE("SceneParser supports JSON and XML", "[scene_loader]")
{
	const std::string json_scene = R"json(
{
  "version": "1.0",
  "settings": {
    "output": {
      "simulation_name": "json_scene",
      "enable_output": false
    },
    "simulation": {
      "init_frictional_contact": false
    }
  },
  "materials": {
    "cloth": {
      "type": "surface",
      "density": 0.2,
      "youngs_modulus": 1000.0
    }
  },
  "objects": {
    "deformables": [
      {
        "id": "cloth_1",
        "type": "surface",
        "material": "cloth",
        "geometry": {
          "type": "grid",
          "size": [1.0, 1.0],
          "subdivisions": [1, 1]
        }
      }
    ],
    "rigidbodies": [
      {
        "id": "floor",
        "label": "floor",
        "geometry": {
          "type": "box",
          "size": [2.0, 2.0, 0.1]
        },
        "mass": 1.0,
        "fixed": true
      }
    ]
  }
}
)json";

	const std::string xml_scene = R"xml(
<?xml version="1.0" encoding="UTF-8"?>
<scene version="1.0">
  <settings>
    <output simulation_name="xml_scene" enable_output="false"/>
    <simulation init_frictional_contact="false"/>
  </settings>
  <materials>
    <material id="cloth" type="surface">
      <density value="0.2"/>
      <youngs_modulus value="1000.0"/>
    </material>
  </materials>
  <objects>
    <deformables>
      <deformable id="cloth_1" type="surface" material="cloth">
        <geometry type="grid">
          <size x="1.0" y="1.0"/>
          <subdivisions nx="1" ny="1"/>
        </geometry>
      </deformable>
    </deformables>
    <rigidbodies>
      <rigidbody id="floor" label="floor" mass="1.0" fixed="true">
        <geometry type="box">
          <size x="2.0" y="2.0" z="0.1"/>
        </geometry>
      </rigidbody>
    </rigidbodies>
  </objects>
</scene>
)xml";

	const auto json_result = nexdyndiff::scene::SceneParser::ParseText(json_scene, nexdyndiff::scene::SceneFileFormat::Json);
	REQUIRE(json_result.success());
	CHECK(json_result.description.deformables.size() == 1);
	CHECK(json_result.description.rigid_bodies.size() == 1);
	CHECK(json_result.description.settings.output.simulation_name == "json_scene");

	const auto xml_result = nexdyndiff::scene::SceneParser::ParseText(xml_scene, nexdyndiff::scene::SceneFileFormat::Xml);
	REQUIRE(xml_result.success());
	CHECK(xml_result.description.deformables.size() == 1);
	CHECK(xml_result.description.rigid_bodies.size() == 1);
	CHECK(xml_result.description.settings.output.simulation_name == "xml_scene");
}

TEST_CASE("SceneLoader builds transforms and handlers from JSON", "[scene_loader]")
{
	const std::filesystem::path tmp_dir = MakeTempDir();
	const std::filesystem::path scene_path = tmp_dir / "scene.json";

	const std::string scene = R"json(
{
  "version": "1.0",
  "settings": {
    "output": {
      "simulation_name": "build_json",
      "output_directory": "./output",
      "codegen_directory": "./codegen",
      "enable_output": false
    },
    "simulation": {
      "init_frictional_contact": false,
      "gravity": [0.0, 0.0, 0.0]
    },
    "execution": {
      "end_simulation_time": 0.01
    }
  },
  "materials": {
    "cloth": {
      "type": "surface",
      "density": 0.2,
      "youngs_modulus": 1000.0
    }
  },
  "objects": {
    "deformables": [
      {
        "id": "cloth_1",
        "type": "surface",
        "label": "cloth",
        "material": "cloth",
        "geometry": {
          "type": "grid",
          "size": [1.0, 1.0],
          "subdivisions": [1, 1]
        },
        "transform": {
          "rotation": [0.0, 0.0, 90.0],
          "translation": [1.0, 2.0, 3.0]
        }
      }
    ],
    "rigidbodies": [
      {
        "id": "floor",
        "label": "floor",
        "geometry": {
          "type": "box",
          "size": [1.0, 1.0, 1.0]
        },
        "mass": 1.0,
        "fixed": true,
        "transform": {
          "translation": [0.0, 0.0, -0.5]
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
        "center": [1.5, 1.5, 3.0],
        "halfsize": [0.01, 0.01, 0.01]
      },
      "params": {
        "stiffness": 1000000.0,
        "tolerance": 0.001
      }
    },
    {
      "type": "rigidbody_constraint",
      "constraint_type": "fix",
      "target": "floor"
    }
  ]
}
)json";

	WriteFile(scene_path, scene);

	const auto load = nexdyndiff::scene::SceneLoader::Load(scene_path);
	REQUIRE(load.success());
	REQUIRE(load.build_result.point_sets.count("cloth_1") == 1);
	REQUIRE(load.build_result.rigid_bodies.count("floor") == 1);

	const auto& cloth = load.build_result.point_sets.at("cloth_1");
	const Eigen::Vector3d p0 = cloth.get_position(0);
	CHECK_THAT(p0.x(), WithinAbs(1.5, 1e-8));
	CHECK_THAT(p0.y(), WithinAbs(1.5, 1e-8));
	CHECK_THAT(p0.z(), WithinAbs(3.0, 1e-8));

	const auto& floor = load.build_result.rigid_bodies.at("floor");
	const Eigen::Vector3d floor_t = floor.get_translation();
	CHECK_THAT(floor_t.x(), WithinAbs(0.0, 1e-8));
	CHECK_THAT(floor_t.y(), WithinAbs(0.0, 1e-8));
	CHECK_THAT(floor_t.z(), WithinAbs(-0.5, 1e-8));
}

TEST_CASE("SceneLoader fails on unknown material", "[scene_loader]")
{
	const std::filesystem::path tmp_dir = MakeTempDir();
	const std::filesystem::path scene_path = tmp_dir / "invalid.json";

	const std::string scene = R"json(
{
  "settings": {
    "output": {
      "simulation_name": "invalid",
      "enable_output": false
    },
    "simulation": {
      "init_frictional_contact": false
    }
  },
  "objects": {
    "deformables": [
      {
        "id": "bad_obj",
        "type": "surface",
        "material": "unknown_material",
        "geometry": {
          "type": "grid",
          "size": [1.0, 1.0],
          "subdivisions": [1, 1]
        }
      }
    ]
  }
}
)json";

	WriteFile(scene_path, scene);

	const auto load = nexdyndiff::scene::SceneLoader::Load(scene_path);
	CHECK_FALSE(load.success());
	REQUIRE_FALSE(load.parse_result.errors.empty());
	CHECK_THAT(load.parse_result.errors.front().message, ContainsSubstring("Unknown material"));
}

TEST_CASE("SceneLoader scripted events update gravity and support transform_bc", "[scene_loader]")
{
	const std::filesystem::path tmp_dir = MakeTempDir();
	const std::filesystem::path scene_path = tmp_dir / "events.json";

	const std::string scene = R"json(
{
  "settings": {
    "output": {
      "simulation_name": "events",
      "enable_output": false
    },
    "simulation": {
      "init_frictional_contact": false,
      "gravity": [0.0, 0.0, 0.0]
    },
    "execution": {
      "end_simulation_time": 0.05
    }
  },
  "materials": {
    "cloth": {
      "type": "surface",
      "density": 0.2,
      "youngs_modulus": 1000.0
    }
  },
  "objects": {
    "deformables": [
      {
        "id": "cloth_1",
        "type": "surface",
        "material": "cloth",
        "geometry": {
          "type": "grid",
          "size": [1.0, 1.0],
          "subdivisions": [1, 1]
        }
      }
    ]
  },
  "boundary_conditions": [
    {
      "id": "bc_fixed",
      "type": "prescribed_position",
      "target": "cloth_1",
      "selector": {
        "type": "inside_aabb",
        "center": [0.0, 0.0, 0.0],
        "halfsize": [0.01, 0.01, 0.01]
      }
    }
  ],
  "scripted_events": [
    {
      "type": "time_event",
      "start_time": 0.0,
      "end_time": 1.0,
      "action": {
        "type": "set_gravity",
        "gravity": [1.0, 2.0, 3.0],
        "blend": "step"
      }
    },
    {
      "type": "time_event",
      "start_time": 0.0,
      "end_time": 1.0,
      "target": "bc_fixed",
      "action": {
        "type": "transform_bc",
        "angular_velocity": 90.0,
        "axis": [0.0, 0.0, 1.0]
      }
    }
  ]
}
)json";

	WriteFile(scene_path, scene);
	auto load = nexdyndiff::scene::SceneLoader::Load(scene_path);
	REQUIRE(load.success());
	REQUIRE(load.simulation != nullptr);
	CHECK(load.parse_result.description.settings.output.output_directory == (tmp_dir / "output").string());
	CHECK(load.parse_result.description.settings.output.codegen_directory == (tmp_dir / "codegen").string());

	load.simulation->run_one_time_step();
	const Eigen::Vector3d gravity = load.simulation->get_gravity();
	CHECK_THAT(gravity.x(), WithinAbs(1.0, 1e-9));
	CHECK_THAT(gravity.y(), WithinAbs(2.0, 1e-9));
	CHECK_THAT(gravity.z(), WithinAbs(3.0, 1e-9));
}

TEST_CASE("SceneLoader async load and cache invalidation on file change", "[scene_loader]")
{
	const std::filesystem::path tmp_dir = MakeTempDir();
	const std::filesystem::path scene_path = tmp_dir / "cache.json";

	const std::string valid_scene = R"json(
{
  "settings": {
    "output": { "simulation_name": "cache_ok", "enable_output": false },
    "simulation": { "init_frictional_contact": false }
  },
  "objects": {
    "rigidbodies": [
      {
        "id": "box_1",
        "geometry": { "type": "box", "size": [1.0, 1.0, 1.0] },
        "mass": 1.0
      }
    ]
  }
}
)json";
	WriteFile(scene_path, valid_scene);

	std::atomic<bool> callback_called{ false };
	auto future = nexdyndiff::scene::SceneLoader::LoadAsync(
		scene_path,
		nexdyndiff::scene::LoadOptions(),
		[&callback_called](const nexdyndiff::scene::LoadResult& result) {
			callback_called = result.success();
		});
	auto async_result = future.get();
	REQUIRE(async_result.success());
	CHECK(callback_called.load());
	CHECK(async_result.parse_result.description.settings.output.output_directory == (tmp_dir / "output").string());
	CHECK(async_result.parse_result.description.settings.output.codegen_directory == (tmp_dir / "codegen").string());

	auto parse_result_1 = nexdyndiff::scene::SceneLoader::ParseOnly(scene_path);
	REQUIRE(parse_result_1.success());

	std::this_thread::sleep_for(std::chrono::milliseconds(20));
	WriteFile(scene_path, "{ invalid json }");

	auto parse_result_2 = nexdyndiff::scene::SceneLoader::ParseOnly(scene_path);
	CHECK_FALSE(parse_result_2.success());
	REQUIRE_FALSE(parse_result_2.errors.empty());
}
