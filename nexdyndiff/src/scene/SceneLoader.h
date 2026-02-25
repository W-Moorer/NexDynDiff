#pragma once

#include <functional>
#include <filesystem>
#include <future>
#include <memory>
#include <optional>
#include <vector>

#include "../models/Simulation.h"
#include "SceneBuilder.h"
#include "SceneCache.h"
#include "SceneParser.h"

namespace nexdyndiff::scene
{
	struct LoadOptions
	{
		bool validate = true;
		bool use_cache = true;
		std::filesystem::path base_path;
	};

	struct LoadResult
	{
		std::unique_ptr<Simulation> simulation;
		ParseResult parse_result;
		BuildResult build_result;

		bool success() const
		{
			return simulation != nullptr && parse_result.success() && build_result.success;
		}

		std::vector<SceneError> all_errors() const
		{
			std::vector<SceneError> output = parse_result.errors;
			output.insert(output.end(), build_result.errors.begin(), build_result.errors.end());
			return output;
		}
	};

	class SceneLoader
	{
	public:
		static LoadResult Load(const std::filesystem::path& file_path, const LoadOptions& options = LoadOptions());
		static std::future<LoadResult> LoadAsync(
			const std::filesystem::path& file_path,
			const LoadOptions& options = LoadOptions(),
			std::function<void(const LoadResult&)> on_complete = nullptr);
		static ParseResult ParseOnly(const std::filesystem::path& file_path);
	};
}
