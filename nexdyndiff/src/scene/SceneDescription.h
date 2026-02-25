#pragma once

#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "../core/Settings.h"

namespace nexdyndiff::scene
{
	struct SceneError
	{
		std::string path;
		std::string message;
	};

	struct TransformDefinition
	{
		Eigen::Vector3d translation = Eigen::Vector3d::Zero();
		Eigen::Vector3d rotation_deg = Eigen::Vector3d::Zero();
	};

	struct GeometryDefinition
	{
		std::string type;
		std::filesystem::path path;
		std::string generator;

		Eigen::Vector2d center2 = Eigen::Vector2d::Zero();
		Eigen::Vector2d size2 = Eigen::Vector2d::Ones();
		std::array<int, 2> subdivisions2 = { 1, 1 };

		Eigen::Vector3d center3 = Eigen::Vector3d::Zero();
		Eigen::Vector3d size3 = Eigen::Vector3d::Ones();
		std::array<int, 3> subdivisions3 = { 1, 1, 1 };

		Eigen::Vector3d begin = Eigen::Vector3d::Zero();
		Eigen::Vector3d end = Eigen::Vector3d(1.0, 0.0, 0.0);
		int n_segments = 1;

		double radius = 0.5;
		double height = 1.0;
		double outer_radius = 1.0;
		double inner_radius = 0.25;
		int slices = 16;
		int stacks = 8;
		int subdivisions = 2;
	};

	struct MaterialDefinition
	{
		std::string id;
		std::string type;

		std::optional<double> density;
		std::optional<double> youngs_modulus;
		std::optional<double> poisson_ratio;
		std::optional<double> damping;
		std::optional<double> strain_limit;
		std::optional<double> bending_stiffness;
		std::optional<double> section_radius;
		std::optional<bool> elasticity_only;

		std::optional<double> contact_thickness;
		std::optional<double> friction;
	};

	struct DeformableDefinition
	{
		std::string id;
		std::string type;
		std::string label;
		std::string material_id;
		GeometryDefinition geometry;
		TransformDefinition transform;
		bool contact_enabled = true;
		std::optional<double> contact_thickness;
		std::optional<double> friction;
	};

	struct RigidBodyDefinition
	{
		std::string id;
		std::string label;
		double mass = 1.0;
		bool fixed = false;
		GeometryDefinition geometry;
		TransformDefinition transform;
		std::optional<double> contact_thickness;
		std::optional<double> friction;
	};

	struct SelectorDefinition
	{
		std::string type = "inside_aabb";
		Eigen::Vector3d center = Eigen::Vector3d::Zero();
		Eigen::Vector3d halfsize = Eigen::Vector3d::Constant(1e-3);
	};

	struct BoundaryConditionDefinition
	{
		std::string id;
		std::string type;
		std::string target;
		std::string constraint_type;
		SelectorDefinition selector;
		double stiffness = 1e7;
		double tolerance = 1e-3;

		std::string body_a;
		std::string body_b;
		Eigen::Vector3d pivot = Eigen::Vector3d::Zero();
		Eigen::Vector3d axis = Eigen::Vector3d::UnitY();
		double target_distance = 0.0;
		double min_limit = 0.0;
		double max_limit = 0.0;
		double damping = 0.0;
		double target_velocity = 0.0;
		double max_force = 0.0;
		double admissible_angle_deg = 0.0;
	};

	struct ScriptedEventActionDefinition
	{
		std::string type;
		std::string blend = "step";

		Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
		Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
		Eigen::Vector3d translation = Eigen::Vector3d::Zero();
		Eigen::Vector3d translation_velocity = Eigen::Vector3d::Zero();

		double angular_velocity = 0.0;
	};

	struct ScriptedEventDefinition
	{
		std::string type;
		std::string id;
		std::string target;
		double start_time = 0.0;
		double end_time = 0.0;
		ScriptedEventActionDefinition action;
	};

	struct ContactGlobalDefinition
	{
		std::optional<double> default_contact_thickness;
		std::optional<double> min_contact_stiffness;
		std::optional<double> friction_stick_slide_threshold;
	};

	struct ContactPairDefinition
	{
		std::string object1;
		std::string object2;
		std::optional<double> friction;
		bool enabled = true;
	};

	struct DisabledContactPairDefinition
	{
		std::string object1;
		std::string object2;
	};

	struct SceneDescription
	{
		std::string version = "1.0";
		std::string name;
		core::Settings settings;
		std::vector<MaterialDefinition> materials;
		std::vector<DeformableDefinition> deformables;
		std::vector<RigidBodyDefinition> rigid_bodies;
		std::vector<BoundaryConditionDefinition> boundary_conditions;
		ContactGlobalDefinition contact_global;
		std::vector<ContactPairDefinition> contact_pairs;
		std::vector<DisabledContactPairDefinition> disabled_contact_pairs;
		std::vector<ScriptedEventDefinition> scripted_events;
	};
}
