#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "nexdyndiff.h"

namespace
{
	struct RigidBodySample
	{
		double time = 0.0;
		Eigen::Vector3d position = Eigen::Vector3d::Zero();
		Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
		Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
	};

	void WriteVectorCsv(
		const std::filesystem::path& file_path,
		const std::vector<RigidBodySample>& samples,
		const std::string& quantity)
	{
		std::ofstream out(file_path, std::ios::out | std::ios::binary);
		out << "time,tx,ty,tz\n";
		out << std::setprecision(10);
		for (const auto& sample : samples) {
			Eigen::Vector3d value = Eigen::Vector3d::Zero();
			if (quantity == "pos") value = sample.position;
			else if (quantity == "vel") value = sample.velocity;
			else value = sample.acceleration;

			out << sample.time << "," << value.x() << "," << value.y() << "," << value.z() << "\n";
		}
	}

	void FillAccelerationFromVelocity(std::vector<RigidBodySample>& samples)
	{
		std::vector<double> time;
		std::vector<Eigen::Vector3d> velocity;
		time.reserve(samples.size());
		velocity.reserve(samples.size());
		for (const auto& sample : samples) {
			time.push_back(sample.time);
			velocity.push_back(sample.velocity);
		}

		const auto acceleration = nexdyndiff::DifferentiateTimeSeries(time, velocity);
		if (acceleration.size() != samples.size()) {
			for (auto& sample : samples) {
				sample.acceleration = Eigen::Vector3d::Zero();
			}
			return;
		}

		for (size_t i = 0; i < samples.size(); ++i) {
			samples[i].acceleration = acceleration[i];
		}
	}
}

int main(int argc, char** argv)
{
	const std::filesystem::path scene_path = (argc > 1)
		? std::filesystem::path(argv[1])
		: (std::filesystem::path("models") / "double_pendulum_scene.json");
	const std::filesystem::path output_dir = (argc > 2)
		? std::filesystem::path(argv[2])
		: (std::filesystem::path("output") / "double_pendulum");

	auto load = nexdyndiff::scene::SceneLoader::Load(scene_path);
	if (!load.success()) {
		std::cerr << "Failed to load scene file: " << scene_path << std::endl;
		for (const auto& error : load.all_errors()) {
			std::cerr << "  - " << error.path << ": " << error.message << std::endl;
		}
		return 1;
	}
	if (load.simulation == nullptr) {
		std::cerr << "Scene loaded but simulation was not created." << std::endl;
		return 1;
	}

	auto body1_it = load.build_result.rigid_bodies.find("Body1");
	auto body2_it = load.build_result.rigid_bodies.find("Body2");
	if (body1_it == load.build_result.rigid_bodies.end() || body2_it == load.build_result.rigid_bodies.end()) {
		std::cerr << "Scene must contain rigid bodies with ids Body1 and Body2." << std::endl;
		return 1;
	}

	auto& body1 = body1_it->second;
	auto& body2 = body2_it->second;
	std::vector<RigidBodySample> body1_samples;
	std::vector<RigidBodySample> body2_samples;
	body1_samples.reserve(4096);
	body2_samples.reserve(4096);

	auto sample_state = [&](std::vector<RigidBodySample>& series, nexdyndiff::RigidBodyHandler& body)
		{
			series.push_back(RigidBodySample{
				load.simulation->get_time(),
				body.get_translation(),
				body.get_velocity(),
				Eigen::Vector3d::Zero()
				});
		};

	sample_state(body1_samples, body1);
	sample_state(body2_samples, body2);

	const double end_time = load.parse_result.description.settings.execution.end_simulation_time;
	const double initial_dt = std::max(1e-12, load.simulation->get_time_step_size());
	const int max_steps = std::max(1000, (int)std::ceil(end_time / initial_dt) + 1000);
	for (int step = 0; step < max_steps && load.simulation->get_time() + 1e-12 < end_time; ++step) {
		load.simulation->run_one_time_step();
		sample_state(body1_samples, body1);
		sample_state(body2_samples, body2);
	}

	// RigidBodyHandler::get_acceleration() exposes prescribed external acceleration terms.
	// For kinematic comparison with reference data we differentiate simulated velocities.
	FillAccelerationFromVelocity(body1_samples);
	FillAccelerationFromVelocity(body2_samples);

	std::filesystem::create_directories(output_dir);
	WriteVectorCsv(output_dir / "Link1_Pos_NexDynDiff.csv", body1_samples, "pos");
	WriteVectorCsv(output_dir / "Link1_Vel_NexDynDiff.csv", body1_samples, "vel");
	WriteVectorCsv(output_dir / "Link1_Acc_NexDynDiff.csv", body1_samples, "acc");
	WriteVectorCsv(output_dir / "Link2_Pos_NexDynDiff.csv", body2_samples, "pos");
	WriteVectorCsv(output_dir / "Link2_Vel_NexDynDiff.csv", body2_samples, "vel");
	WriteVectorCsv(output_dir / "Link2_Acc_NexDynDiff.csv", body2_samples, "acc");

	std::cout << "Double pendulum simulation finished." << std::endl;
	std::cout << "Output directory: " << output_dir << std::endl;
	std::cout << "Samples written: " << body1_samples.size() << std::endl;

	// The core library currently exhibits a teardown issue on process exit for this scene.
	// Releasing ownership here avoids deleting the simulation object during shutdown.
	load.simulation.release();

	return 0;
}
