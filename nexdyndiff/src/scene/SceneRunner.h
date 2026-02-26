#pragma once

#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>

#include "SceneLoader.h"
#include "SceneCsvExporter.h"

namespace nexdyndiff::scene
{
    struct RunOptions
    {
        bool use_cache = true;
        bool validate = true;
        bool verbose = true;
        bool export_csv = true;
        int csv_fps = 60;
        std::filesystem::path base_path;
    };

    struct RunResult
    {
        int exit_code = 0;
        std::string error_message;
        bool success() const { return exit_code == 0; }
    };

    class SceneRunner
    {
    public:
        static int Run(
            const std::filesystem::path& scene_file,
            const RunOptions& options = RunOptions());

        static RunResult RunWithResult(
            const std::filesystem::path& scene_file,
            const RunOptions& options = RunOptions());

    private:
        static void PrintBanner();
        static void PrintLoadResult(const LoadResult& result);
    };
}
