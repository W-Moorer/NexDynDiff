#include <algorithm>
#include <filesystem>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "nexdyndiff.h"

#include <catch2/catch_test_macros.hpp>

namespace
{
	struct RigidState
	{
		Eigen::Vector3d translation = Eigen::Vector3d::Zero();
		Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
		Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
		Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
	};

	void RunUntilEnd(nexdyndiff::Simulation& simulation, double end_time)
	{
		const int max_steps = 20000;
		int step = 0;
		while (simulation.get_time() + 1e-12 < end_time && step < max_steps) {
			simulation.run_one_time_step();
			step++;
		}
	}

	nexdyndiff::Settings MakeTemplateSettings(const std::string& name)
	{
		const std::filesystem::path repo_root = std::filesystem::path(__FILE__).parent_path().parent_path();
		nexdyndiff::Settings settings = nexdyndiff::Settings();
		settings.output.simulation_name = name;
		settings.output.output_directory = (repo_root / "output" / "rb_constraints_baseline").string();
		settings.output.codegen_directory = (repo_root / "build" / "codegen").string();
		settings.output.enable_output = false;
		settings.execution.end_simulation_time = 5.0;
		settings.simulation.init_frictional_contact = false;
		return settings;
	}

	nexdyndiff::RigidBodyHandler AddBox(nexdyndiff::Simulation& simulation, const Eigen::Vector3d& translation)
	{
		const double mass = 1.0;
		const double size = 0.1;
		auto [vertices, triangles, box] = simulation.presets->rigidbodies->add_box("box", mass, size);
		box.rigidbody.set_translation(translation);
		return box.rigidbody;
	}

	std::unordered_map<std::string, RigidState> CaptureStates(const std::vector<nexdyndiff::RigidBodyHandler>& boxes)
	{
		std::unordered_map<std::string, RigidState> states;
		for (size_t i = 0; i < boxes.size(); ++i) {
			RigidState state;
			state.translation = boxes[i].get_translation();
			state.rotation = boxes[i].get_rotation_matrix();
			state.velocity = boxes[i].get_velocity();
			state.angular_velocity = boxes[i].get_angular_velocity();
			states.insert_or_assign("box" + std::to_string(i), state);
		}
		return states;
	}

	std::unordered_map<std::string, RigidState> RunBaselineScenario(const std::string& scenario_name)
	{
		nexdyndiff::Simulation simulation(MakeTemplateSettings(scenario_name));

		std::vector<nexdyndiff::RigidBodyHandler> boxes;
		boxes.push_back(AddBox(simulation, Eigen::Vector3d::Zero()));
		simulation.rigidbodies->add_constraint_fix(boxes[0]);

		if (scenario_name == "ball_joint") {
			for (int i = 1; i < 10; ++i) {
				auto curr = AddBox(simulation, { 0.1 * i, 0.0, 0.0 });
				const Eigen::Vector3d pivot = (i % 2)
					? Eigen::Vector3d{ 0.05 + (i - 1) * 0.1, -0.05, -0.05 }
					: Eigen::Vector3d{ 0.05 + (i - 1) * 0.1, 0.05, 0.05 };
				simulation.rigidbodies->add_constraint_point(boxes.back(), curr, pivot);
				boxes.push_back(curr);
			}
		}
		else if (scenario_name == "point_on_axis") {
			for (int i = 1; i < 10; ++i) {
				auto curr = AddBox(simulation, { 0.1 * i, 0.0, 0.0 });
				const Eigen::Vector3d pivot = (i % 2)
					? Eigen::Vector3d{ 0.05 + (i - 1) * 0.1, -0.05, -0.05 }
					: Eigen::Vector3d{ 0.05 + (i - 1) * 0.1, 0.05, 0.05 };
				simulation.rigidbodies->add_constraint_point(boxes.back(), curr, pivot);
				boxes.push_back(curr);
			}
			simulation.rigidbodies->add_constraint_point_on_axis(boxes[0], boxes.back(), boxes.back().get_translation(), Eigen::Vector3d::UnitX());
		}
		else if (scenario_name == "relative_direction_lock") {
			for (int i = 1; i < 10; ++i) {
				auto curr = AddBox(simulation, { 0.1 * i, 0.0, 0.0 });
				const Eigen::Vector3d pivot = (i % 2)
					? Eigen::Vector3d{ 0.05 + (i - 1) * 0.1, -0.05, -0.05 }
					: Eigen::Vector3d{ 0.05 + (i - 1) * 0.1, 0.05, 0.05 };
				simulation.rigidbodies->add_constraint_point(boxes.back(), curr, pivot);
				if (i != 1) {
					simulation.rigidbodies->add_constraint_direction(boxes.back(), curr, Eigen::Vector3d::UnitY());
				}
				boxes.push_back(curr);
			}
		}
		else {
			auto box1 = AddBox(simulation, Eigen::Vector3d::Zero());
			boxes.push_back(box1);

			if (scenario_name == "distance") {
				box1.set_translation({ 0.5, 0.0, 0.0 });
				simulation.rigidbodies->add_constraint_distance(boxes[0], box1, boxes[0].get_translation(), box1.get_translation() - 0.05 * Eigen::Vector3d::Ones());
			}
			else if (scenario_name == "distance_limits") {
				box1.set_translation({ 0.5, 0.0, 0.0 });
				simulation.rigidbodies->add_constraint_distance_limits(boxes[0], box1, boxes[0].get_translation(), box1.get_translation() - 0.05 * Eigen::Vector3d::Ones(), 0.44, 0.8);
			}
			else if (scenario_name == "angle_limits") {
				box1.set_translation({ 0.5, 0.0, 0.0 });
				simulation.rigidbodies->add_constraint_distance_limits(boxes[0], box1, boxes[0].get_translation(), box1.get_translation() - 0.05 * Eigen::Vector3d::Ones(), 0.44, 0.8);
				simulation.rigidbodies->add_constraint_angle_limit(boxes[0], box1, Eigen::Vector3d::UnitZ(), 20.0);
			}
			else if (scenario_name == "spring") {
				box1.set_translation({ 0.5, 0.0, 0.0 });
				simulation.rigidbodies->add_constraint_spring(boxes[0], box1, boxes[0].get_translation(), box1.get_translation() - 0.05 * Eigen::Vector3d::Ones(), 5.0, 1.0);
			}
			else if (scenario_name == "attachment") {
				box1.set_translation({ 0.1, 0.0, 0.0 });
				simulation.rigidbodies->add_constraint_attachment(boxes[0], box1);
			}
			else if (scenario_name == "point_with_angle_limit") {
				box1.set_translation({ 0.1, 0.0, 0.0 });
				simulation.rigidbodies->add_constraint_point_with_angle_limit(boxes[0], box1, { 0.05, 0.0, 0.0 }, Eigen::Vector3d::UnitX(), 30.0);
			}
			else if (scenario_name == "hinge") {
				box1.set_translation({ 0.1, 0.0, 0.0 });
				simulation.rigidbodies->add_constraint_hinge(boxes[0], box1, { 0.05, 0.0, -0.05 }, Eigen::Vector3d::UnitY());
			}
			else if (scenario_name == "hinge_with_limits") {
				box1.set_translation({ 0.1, 0.0, 0.0 });
				simulation.rigidbodies->add_constraint_hinge_with_angle_limit(boxes[0], box1, { 0.05, 0.0, -0.05 }, Eigen::Vector3d::UnitY(), 30.0);
			}
			else if (scenario_name == "spring_with_limits") {
				box1.set_translation({ 0.1, 0.0, 0.0 });
				simulation.rigidbodies->add_constraint_spring_with_limits(boxes[0], box1, boxes[0].get_translation(), box1.get_translation(), 20.0, 0.09, 0.8);
			}
			else if (scenario_name == "slider") {
				box1.set_translation({ 0.1, 0.0, 0.0 });
				simulation.rigidbodies->add_constraint_spring(boxes[0], box1, boxes[0].get_translation(), box1.get_translation(), 0.2);
				simulation.rigidbodies->add_constraint_slider(boxes[0], box1, { 0.05, -0.05, 0.05 }, { 1.0, 0.0, -1.0 });
			}
			else if (scenario_name == "prismatic_slider") {
				box1.set_translation({ 0.1, 0.0, 0.0 });
				simulation.rigidbodies->add_constraint_spring(boxes[0], box1, boxes[0].get_translation(), box1.get_translation(), 0.2);
				simulation.rigidbodies->add_constraint_prismatic_slider(boxes[0], box1, { 0.05, -0.05, 0.05 }, { 1.0, 0.0, -1.0 });
			}
			else if (scenario_name == "prismatic_press") {
				box1.set_translation({ 0.0, 0.0, -0.5 });
				simulation.rigidbodies->add_constraint_prismatic_press(boxes[0], box1, boxes[0].get_translation(), Eigen::Vector3d::UnitZ(), 0.2, 20.0);
				simulation.rigidbodies->add_constraint_spring(boxes[0], box1, boxes[0].get_translation(), box1.get_translation(), 8.0, 0.5);
			}
			else if (scenario_name == "motor") {
				box1.set_translation({ 0.2, 0.0, 0.0 });
				simulation.rigidbodies->add_constraint_motor(boxes[0], box1, boxes[0].get_translation(), Eigen::Vector3d::UnitY(), -1.0, 10.0);
			}
			else {
				FAIL("Unknown baseline scenario: " + scenario_name);
			}
		}

		RunUntilEnd(simulation, simulation.get_settings().execution.end_simulation_time);
		return CaptureStates(boxes);
	}

	std::unordered_map<std::string, RigidState> RunScene(const std::filesystem::path& scene_path)
	{
		auto load = nexdyndiff::scene::SceneLoader::Load(scene_path);
		REQUIRE(load.success());
		REQUIRE(load.simulation != nullptr);
		RunUntilEnd(*load.simulation, load.parse_result.description.settings.execution.end_simulation_time);

		std::unordered_map<std::string, RigidState> states;
		for (const auto& kv : load.build_result.rigid_bodies) {
			if (kv.first.rfind("box", 0) != 0) continue;
			RigidState state;
			state.translation = kv.second.get_translation();
			state.rotation = kv.second.get_rotation_matrix();
			state.velocity = kv.second.get_velocity();
			state.angular_velocity = kv.second.get_angular_velocity();
			states.insert_or_assign(kv.first, state);
		}
		return states;
	}

	void AccumulateErrors(
		const std::unordered_map<std::string, RigidState>& lhs,
		const std::unordered_map<std::string, RigidState>& rhs,
		double& max_translation_error,
		double& max_rotation_error,
		double& max_velocity_error,
		double& max_angular_velocity_error)
	{
		for (const auto& [id, lhs_state] : lhs) {
			auto it = rhs.find(id);
			REQUIRE(it != rhs.end());
			const auto& rhs_state = it->second;

			max_translation_error = std::max(max_translation_error, (lhs_state.translation - rhs_state.translation).norm());
			max_rotation_error = std::max(max_rotation_error, (lhs_state.rotation - rhs_state.rotation).norm());
			max_velocity_error = std::max(max_velocity_error, (lhs_state.velocity - rhs_state.velocity).norm());
			max_angular_velocity_error = std::max(max_angular_velocity_error, (lhs_state.angular_velocity - rhs_state.angular_velocity).norm());
		}
	}
}

TEST_CASE("rb_constraint scenes match C++ baselines", "[scene_loader][rb_constraints][baseline]")
{
	const auto repo_root = std::filesystem::path(__FILE__).parent_path().parent_path();
	const std::filesystem::path models_json_dir = repo_root / "models" / "rb_constraints";
	const std::filesystem::path models_xml_dir = repo_root / "models" / "rb_constraints_xml";

	const std::vector<std::string> scenario_names = {
		"ball_joint",
		"point_on_axis",
		"distance",
		"distance_limits",
		"relative_direction_lock",
		"angle_limits",
		"spring",
		"attachment",
		"point_with_angle_limit",
		"hinge",
		"hinge_with_limits",
		"spring_with_limits",
		"slider",
		"prismatic_slider",
		"prismatic_press",
		"motor"
	};

	for (const auto& scenario_name : scenario_names) {
		SECTION(scenario_name) {
			const std::filesystem::path json_scene_path = models_json_dir / (scenario_name + ".json");
			const std::filesystem::path xml_scene_path = models_xml_dir / (scenario_name + ".xml");
			REQUIRE(std::filesystem::exists(json_scene_path));
			REQUIRE(std::filesystem::exists(xml_scene_path));

			const auto baseline_states = RunBaselineScenario(scenario_name);
			const auto json_states = RunScene(json_scene_path);
			const auto xml_states = RunScene(xml_scene_path);

			REQUIRE(baseline_states.size() == json_states.size());
			REQUIRE(baseline_states.size() == xml_states.size());

			double max_translation_error_cpp_json = 0.0;
			double max_rotation_error_cpp_json = 0.0;
			double max_velocity_error_cpp_json = 0.0;
			double max_angular_velocity_error_cpp_json = 0.0;

			double max_translation_error_cpp_xml = 0.0;
			double max_rotation_error_cpp_xml = 0.0;
			double max_velocity_error_cpp_xml = 0.0;
			double max_angular_velocity_error_cpp_xml = 0.0;

			double max_translation_error_json_xml = 0.0;
			double max_rotation_error_json_xml = 0.0;
			double max_velocity_error_json_xml = 0.0;
			double max_angular_velocity_error_json_xml = 0.0;

			AccumulateErrors(
				baseline_states,
				json_states,
				max_translation_error_cpp_json,
				max_rotation_error_cpp_json,
				max_velocity_error_cpp_json,
				max_angular_velocity_error_cpp_json);

			AccumulateErrors(
				baseline_states,
				xml_states,
				max_translation_error_cpp_xml,
				max_rotation_error_cpp_xml,
				max_velocity_error_cpp_xml,
				max_angular_velocity_error_cpp_xml);

			AccumulateErrors(
				json_states,
				xml_states,
				max_translation_error_json_xml,
				max_rotation_error_json_xml,
				max_velocity_error_json_xml,
				max_angular_velocity_error_json_xml);

			CHECK(max_translation_error_cpp_json < 1e-8);
			CHECK(max_rotation_error_cpp_json < 1e-8);
			CHECK(max_velocity_error_cpp_json < 1e-8);
			CHECK(max_angular_velocity_error_cpp_json < 1e-8);

			CHECK(max_translation_error_cpp_xml < 1e-8);
			CHECK(max_rotation_error_cpp_xml < 1e-8);
			CHECK(max_velocity_error_cpp_xml < 1e-8);
			CHECK(max_angular_velocity_error_cpp_xml < 1e-8);

			CHECK(max_translation_error_json_xml < 1e-10);
			CHECK(max_rotation_error_json_xml < 1e-10);
			CHECK(max_velocity_error_json_xml < 1e-10);
			CHECK(max_angular_velocity_error_json_xml < 1e-10);
		}
	}
}
