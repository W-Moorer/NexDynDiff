#pragma once

#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include "../models/Simulation.h"
#include "SceneDescription.h"

namespace nexdyndiff::scene
{
    struct RigidBodySample
    {
        double time = 0.0;
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    };

    struct CsvExportOptions
    {
        bool enabled = true;
        int fps = 60;
    };

    class SceneCsvExporter
    {
    public:
        SceneCsvExporter(
            const SceneDescription& description,
            const std::unordered_map<std::string, RigidBodyHandler>& rigid_bodies,
            const CsvExportOptions& options = CsvExportOptions());

        void sample(double time);
        void write_to_disk();

    private:
        const SceneDescription& m_description;
        const std::unordered_map<std::string, RigidBodyHandler>& m_rigid_bodies;
        const CsvExportOptions m_options;

        std::unordered_map<std::string, std::vector<RigidBodySample>> m_samples;

        void write_rigid_body_csv(
            const std::string& id,
            const std::vector<RigidBodySample>& samples,
            const std::filesystem::path& output_dir);
    };
}
