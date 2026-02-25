#pragma once

#include <filesystem>
#include <mutex>
#include <string>
#include <unordered_map>

#include "SceneParser.h"

namespace nexdyndiff::scene
{
	class SceneCache
	{
	public:
		static SceneCache& Instance();

		bool Get(const std::filesystem::path& file_path, ParseResult& out_parse_result);
		void Put(const std::filesystem::path& file_path, const ParseResult& parse_result);
		void Invalidate(const std::filesystem::path& file_path);
		void Clear();

	private:
		struct CacheEntry
		{
			ParseResult parse_result;
			std::filesystem::file_time_type last_write_time;
		};

		std::mutex m_mutex;
		std::unordered_map<std::string, CacheEntry> m_entries;
	};
}
