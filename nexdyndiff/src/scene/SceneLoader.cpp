#include "SceneLoader.h"

namespace nexdyndiff::scene
{
	namespace
	{
		void ApplyOutputDefaults(
			core::Settings& settings,
			const std::filesystem::path& file_path,
			const LoadOptions& options)
		{
			const std::filesystem::path base_path = options.base_path.empty()
				? (file_path.has_parent_path() ? file_path.parent_path() : std::filesystem::path())
				: options.base_path;

			if (settings.output.simulation_name.empty()) {
				settings.output.simulation_name = file_path.stem().string().empty()
					? std::string("scene")
					: file_path.stem().string();
			}

			if (settings.output.output_directory.empty()) {
				const std::filesystem::path default_output = base_path.empty()
					? std::filesystem::path("output")
					: (base_path / "output");
				settings.output.output_directory = default_output.string();
			}

			if (settings.output.codegen_directory.empty()) {
				const std::filesystem::path default_codegen = base_path.empty()
					? std::filesystem::path("codegen")
					: (base_path / "codegen");
				settings.output.codegen_directory = default_codegen.string();
			}
		}
	}

	LoadResult SceneLoader::Load(const std::filesystem::path& file_path, const LoadOptions& options)
	{
		LoadResult result;
		if (options.use_cache && SceneCache::Instance().Get(file_path, result.parse_result)) {
			// Cache hit
		}
		else {
			result.parse_result = SceneParser::ParseFile(file_path);
			if (options.use_cache) {
				SceneCache::Instance().Put(file_path, result.parse_result);
			}
		}
		if (!result.parse_result.success()) {
			return result;
		}

		ApplyOutputDefaults(result.parse_result.description.settings, file_path, options);
		result.simulation = std::make_unique<Simulation>(result.parse_result.description.settings);

		const std::filesystem::path base_path = options.base_path.empty()
			? (file_path.has_parent_path() ? file_path.parent_path() : std::filesystem::path())
			: options.base_path;

		SceneBuilder builder(result.parse_result.description, base_path);
		result.build_result = builder.Build(*result.simulation);
		if (!result.build_result.success) {
			result.simulation.reset();
		}

		return result;
	}

	std::future<LoadResult> SceneLoader::LoadAsync(
		const std::filesystem::path& file_path,
		const LoadOptions& options,
		std::function<void(const LoadResult&)> on_complete)
	{
		return std::async(std::launch::async,
			[file_path, options, on_complete]()
			{
				LoadResult result = SceneLoader::Load(file_path, options);
				if (on_complete) {
					on_complete(result);
				}
				return result;
			});
	}

	ParseResult SceneLoader::ParseOnly(const std::filesystem::path& file_path)
	{
		ParseResult parse_result;
		if (SceneCache::Instance().Get(file_path, parse_result)) {
			return parse_result;
		}
		parse_result = SceneParser::ParseFile(file_path);
		SceneCache::Instance().Put(file_path, parse_result);
		return parse_result;
	}
}
