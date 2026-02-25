#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "SceneDescription.h"

namespace nexdyndiff::scene
{
	enum class SceneFileFormat
	{
		Json,
		Xml
	};

	struct ParseResult
	{
		SceneDescription description;
		std::vector<SceneError> errors;

		bool success() const
		{
			return errors.empty();
		}
	};

	class SceneParser
	{
	public:
		static std::optional<SceneFileFormat> DetectFormat(const std::filesystem::path& file_path);
		static ParseResult ParseFile(const std::filesystem::path& file_path);
		static ParseResult ParseText(
			const std::string& text,
			SceneFileFormat format,
			const std::filesystem::path& base_path = std::filesystem::path());
	};
}
