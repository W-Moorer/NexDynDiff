#include "SceneRunner.h"

#include <fmt/format.h>
#include <fmt/color.h>

namespace nexdyndiff::scene
{
    namespace
    {
        void PrintError(const std::string& message)
        {
            fmt::print(fmt::fg(fmt::color::red), "Error: {}\n", message);
        }

        void PrintSuccess(const std::string& message)
        {
            fmt::print(fmt::fg(fmt::color::green), "Success: {}\n", message);
        }

        void PrintInfo(const std::string& message)
        {
            fmt::print(fmt::fg(fmt::color::cyan), "Info: {}\n", message);
        }
    }

    void SceneRunner::PrintBanner()
    {
        fmt::print("\n");
        fmt::print(fmt::fg(fmt::color::yellow), "========================================\n");
        fmt::print(fmt::fg(fmt::color::yellow), "       NexDynDiff Scene Runner         \n");
        fmt::print(fmt::fg(fmt::color::yellow), "========================================\n");
        fmt::print("\n");
    }

    void SceneRunner::PrintLoadResult(const LoadResult& result)
    {
        if (result.success()) {
            PrintSuccess(fmt::format("Scene loaded successfully: {}",
                result.parse_result.description.settings.output.simulation_name));

            if (!result.build_result.point_sets.empty()) {
                PrintInfo(fmt::format("  - Point sets: {}", result.build_result.point_sets.size()));
            }
            if (!result.build_result.rigid_bodies.empty()) {
                PrintInfo(fmt::format("  - Rigid bodies: {}", result.build_result.rigid_bodies.size()));
            }
            if (!result.build_result.contacts.empty()) {
                PrintInfo(fmt::format("  - Contacts: {}", result.build_result.contacts.size()));
            }
            if (!result.build_result.prescribed_positions.empty()) {
                PrintInfo(fmt::format("  - Prescribed positions: {}", result.build_result.prescribed_positions.size()));
            }
        }
        else {
            PrintError("Failed to load scene");
            for (const auto& err : result.all_errors()) {
                fmt::print(fmt::fg(fmt::color::red), "  [{}] {}\n", err.path, err.message);
            }
        }
    }

    int SceneRunner::Run(const std::filesystem::path& scene_file, const RunOptions& options)
    {
        return RunWithResult(scene_file, options).exit_code;
    }

    RunResult SceneRunner::RunWithResult(const std::filesystem::path& scene_file, const RunOptions& options)
    {
        RunResult result;

        if (options.verbose) {
            PrintBanner();
        }

        if (!std::filesystem::exists(scene_file)) {
            result.exit_code = 1;
            result.error_message = fmt::format("Scene file not found: {}", scene_file.string());
            if (options.verbose) {
                PrintError(result.error_message);
            }
            return result;
        }

        LoadOptions load_options;
        load_options.use_cache = options.use_cache;
        load_options.validate = options.validate;
        load_options.base_path = options.base_path;

        if (options.verbose) {
            PrintInfo(fmt::format("Loading scene: {}", scene_file.string()));
        }

        LoadResult load_result = SceneLoader::Load(scene_file, load_options);

        if (options.verbose) {
            PrintLoadResult(load_result);
        }

        if (!load_result.success()) {
            result.exit_code = 2;
            result.error_message = "Failed to parse or build scene";
            return result;
        }

        std::unique_ptr<SceneCsvExporter> csv_exporter;
        if (options.export_csv && !load_result.build_result.rigid_bodies.empty()) {
            CsvExportOptions csv_options;
            csv_options.enabled = true;
            csv_options.fps = options.csv_fps;

            csv_exporter = std::make_unique<SceneCsvExporter>(
                load_result.parse_result.description,
                load_result.build_result.rigid_bodies,
                csv_options);

            if (options.verbose) {
                PrintInfo(fmt::format("CSV export enabled ({} fps)", options.csv_fps));
            }
        }

        if (options.verbose) {
            PrintInfo("Starting simulation...");
            fmt::print("\n");
        }

        const double end_time = load_result.parse_result.description.settings.execution.end_simulation_time;
        const double fps = options.export_csv ? (double)options.csv_fps : 30.0;
        const double frame_time = 1.0 / fps;
        double next_sample_time = 0.0;

        if (csv_exporter) {
            csv_exporter->sample(load_result.simulation->get_time());
        }

        while (load_result.simulation->get_time() < end_time - 1e-12) {
            load_result.simulation->run_one_time_step();

            if (csv_exporter && load_result.simulation->get_time() >= next_sample_time - 1e-12) {
                csv_exporter->sample(load_result.simulation->get_time());
                next_sample_time += frame_time;
            }
        }

        if (csv_exporter) {
            csv_exporter->write_to_disk();
            if (options.verbose) {
                PrintInfo("CSV export completed");
            }
        }

        if (options.verbose) {
            fmt::print("\n");
            PrintSuccess("Simulation completed");
            PrintInfo(fmt::format("Output directory: {}",
                load_result.parse_result.description.settings.output.output_directory));
        }

        result.exit_code = 0;
        return result;
    }
}
