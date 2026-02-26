#include "SceneCsvExporter.h"

#include <fstream>
#include <iomanip>

#include <fmt/format.h>

namespace nexdyndiff::scene
{
    namespace
    {
        void ensure_directory(const std::filesystem::path& dir)
        {
            if (!std::filesystem::exists(dir)) {
                std::filesystem::create_directories(dir);
            }
        }
    }

    SceneCsvExporter::SceneCsvExporter(
        const SceneDescription& description,
        const std::unordered_map<std::string, RigidBodyHandler>& rigid_bodies,
        const CsvExportOptions& options)
        : m_description(description)
        , m_rigid_bodies(rigid_bodies)
        , m_options(options)
    {
    }

    void SceneCsvExporter::sample(double time)
    {
        if (!m_options.enabled || m_rigid_bodies.empty()) {
            return;
        }

        for (const auto& [id, handler] : m_rigid_bodies) {
            RigidBodySample sample;
            sample.time = time;
            sample.position = handler.get_translation();
            sample.rotation = handler.get_rotation_matrix();
            sample.velocity = handler.get_velocity();
            sample.angular_velocity = handler.get_angular_velocity();

            m_samples[id].push_back(sample);
        }
    }

    void SceneCsvExporter::write_to_disk()
    {
        if (!m_options.enabled || m_samples.empty()) {
            return;
        }

        const std::string output_dir = m_description.settings.output.output_directory;
        ensure_directory(output_dir);

        for (const auto& [id, samples] : m_samples) {
            write_rigid_body_csv(id, samples, output_dir);
        }
    }

    void SceneCsvExporter::write_rigid_body_csv(
        const std::string& id,
        const std::vector<RigidBodySample>& samples,
        const std::filesystem::path& output_dir)
    {
        if (samples.empty()) {
            return;
        }

        constexpr int precision = 10;

        std::ofstream pos_file(output_dir / fmt::format("{}_Pos.csv", id), std::ios::out | std::ios::binary);
        pos_file << std::setprecision(precision);
        pos_file << "time,tx,ty,tz\n";
        for (const auto& s : samples) {
            pos_file << s.time << "," << s.position.x() << "," << s.position.y() << "," << s.position.z() << "\n";
        }

        std::ofstream rot_file(output_dir / fmt::format("{}_Rot.csv", id), std::ios::out | std::ios::binary);
        rot_file << std::setprecision(precision);
        rot_file << "time,r00,r01,r02,r10,r11,r12,r20,r21,r22\n";
        for (const auto& s : samples) {
            rot_file << s.time << ","
                     << s.rotation(0, 0) << "," << s.rotation(0, 1) << "," << s.rotation(0, 2) << ","
                     << s.rotation(1, 0) << "," << s.rotation(1, 1) << "," << s.rotation(1, 2) << ","
                     << s.rotation(2, 0) << "," << s.rotation(2, 1) << "," << s.rotation(2, 2) << "\n";
        }

        std::ofstream vel_file(output_dir / fmt::format("{}_Vel.csv", id), std::ios::out | std::ios::binary);
        vel_file << std::setprecision(precision);
        vel_file << "time,vx,vy,vz\n";
        for (const auto& s : samples) {
            vel_file << s.time << "," << s.velocity.x() << "," << s.velocity.y() << "," << s.velocity.z() << "\n";
        }

        std::ofstream ang_vel_file(output_dir / fmt::format("{}_AngVel.csv", id), std::ios::out | std::ios::binary);
        ang_vel_file << std::setprecision(precision);
        ang_vel_file << "time,wx,wy,wz\n";
        for (const auto& s : samples) {
            ang_vel_file << s.time << "," << s.angular_velocity.x() << "," << s.angular_velocity.y() << "," << s.angular_velocity.z() << "\n";
        }
    }
}
