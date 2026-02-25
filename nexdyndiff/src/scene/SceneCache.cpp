#include "SceneCache.h"

namespace
{
	std::string NormalizeKey(const std::filesystem::path& file_path)
	{
		std::error_code ec;
		const auto canonical = std::filesystem::weakly_canonical(file_path, ec);
		if (!ec) {
			return canonical.string();
		}
		return file_path.lexically_normal().string();
	}
}

nexdyndiff::scene::SceneCache& nexdyndiff::scene::SceneCache::Instance()
{
	static SceneCache cache;
	return cache;
}

bool nexdyndiff::scene::SceneCache::Get(const std::filesystem::path& file_path, ParseResult& out_parse_result)
{
	std::error_code ec;
	const auto current_write_time = std::filesystem::last_write_time(file_path, ec);
	if (ec) {
		return false;
	}

	const std::string key = NormalizeKey(file_path);
	std::lock_guard<std::mutex> lock(m_mutex);
	auto it = m_entries.find(key);
	if (it == m_entries.end()) {
		return false;
	}
	if (it->second.last_write_time != current_write_time) {
		m_entries.erase(it);
		return false;
	}

	out_parse_result = it->second.parse_result;
	return true;
}

void nexdyndiff::scene::SceneCache::Put(const std::filesystem::path& file_path, const ParseResult& parse_result)
{
	std::error_code ec;
	const auto current_write_time = std::filesystem::last_write_time(file_path, ec);
	if (ec) {
		return;
	}

	const std::string key = NormalizeKey(file_path);
	std::lock_guard<std::mutex> lock(m_mutex);
	m_entries[key] = CacheEntry{ parse_result, current_write_time };
}

void nexdyndiff::scene::SceneCache::Invalidate(const std::filesystem::path& file_path)
{
	const std::string key = NormalizeKey(file_path);
	std::lock_guard<std::mutex> lock(m_mutex);
	m_entries.erase(key);
}

void nexdyndiff::scene::SceneCache::Clear()
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_entries.clear();
}
