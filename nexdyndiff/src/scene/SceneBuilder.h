#pragma once

#include <filesystem>
#include <unordered_map>
#include <vector>

#include "../models/include.h"
#include "SceneDescription.h"

namespace nexdyndiff::scene
{
	struct BuildResult
	{
		bool success = false;
		std::vector<SceneError> errors;
		std::unordered_map<std::string, PointSetHandler> point_sets;
		std::unordered_map<std::string, RigidBodyHandler> rigid_bodies;
		std::unordered_map<std::string, ContactHandler> contacts;
		std::unordered_map<std::string, EnergyPrescribedPositions::Handler> prescribed_positions;
	};

	class SceneBuilder
	{
	public:
		SceneBuilder(
			const SceneDescription& description,
			const std::filesystem::path& base_path = std::filesystem::path());

		BuildResult Build(Simulation& simulation) const;

	private:
		const SceneDescription& m_description;
		std::filesystem::path m_base_path;
	};
}
