#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "nexdyndiff.h"

#include <catch2/catch_test_macros.hpp>

namespace
{
	std::vector<std::string> split_csv_row(const std::string& row)
	{
		std::vector<std::string> tokens;
		std::stringstream ss(row);
		std::string token;
		while (std::getline(ss, token, ',')) {
			tokens.push_back(token);
		}
		return tokens;
	}

	void read_baseline_ty_csv(
		const std::filesystem::path& csv_path,
		std::vector<double>& out_time,
		std::vector<double>& out_ty)
	{
		std::ifstream in(csv_path, std::ios::in | std::ios::binary);
		REQUIRE(in.is_open());

		std::string line;
		std::getline(in, line); // header
		while (std::getline(in, line)) {
			if (line.empty()) continue;
			auto cols = split_csv_row(line);
			if (cols.size() < 3) continue;
			out_time.push_back(std::stod(cols[0]));
			out_ty.push_back(std::stod(cols[2]));
		}
	}

	double interp_linear(const std::vector<double>& x, const std::vector<double>& y, const double xq)
	{
		if (x.empty()) return 0.0;
		if (xq <= x.front()) return y.front();
		if (xq >= x.back()) return y.back();

		auto it = std::lower_bound(x.begin(), x.end(), xq);
		if (it == x.end()) return y.back();
		const size_t i1 = (size_t)std::distance(x.begin(), it);
		if (i1 == 0) return y.front();
		const size_t i0 = i1 - 1;

		const double x0 = x[i0];
		const double x1 = x[i1];
		const double y0 = y[i0];
		const double y1 = y[i1];
		const double a = (xq - x0) / std::max(1e-15, x1 - x0);
		return y0 * (1.0 - a) + y1 * a;
	}
}

TEST_CASE("basic_contact scene regression against Body2 baseline", "[scene_loader][regression][basic_contact]")
{
	const auto repo_root = std::filesystem::path(__FILE__).parent_path().parent_path();
	const std::filesystem::path scene_json = repo_root / "assets" / "models" / "basic_contact" / "basic_contact.json";
	const std::filesystem::path baseline_csv = repo_root / "assets" / "models" / "basic_contact" / "Body2_Pos.csv";

	REQUIRE(std::filesystem::exists(scene_json));
	REQUIRE(std::filesystem::exists(baseline_csv));

	std::vector<double> baseline_t;
	std::vector<double> baseline_ty;
	read_baseline_ty_csv(baseline_csv, baseline_t, baseline_ty);
	REQUIRE_FALSE(baseline_t.empty());

	auto load = nexdyndiff::scene::SceneLoader::Load(scene_json);
	REQUIRE(load.success());
	REQUIRE(load.simulation != nullptr);
	REQUIRE(load.build_result.rigid_bodies.count("Body2") == 1);

	auto body2 = load.build_result.rigid_bodies.at("Body2");
	std::vector<double> sim_t;
	std::vector<double> sim_ty;
	sim_t.reserve(4096);
	sim_ty.reserve(4096);

	sim_t.push_back(load.simulation->get_time());
	sim_ty.push_back(body2.get_translation().y());

	const double end_time = load.parse_result.description.settings.execution.end_simulation_time;
	const int max_steps = 20000;
	int step = 0;
	while (load.simulation->get_time() + 1e-12 < end_time && step < max_steps) {
		load.simulation->run_one_time_step();
		sim_t.push_back(load.simulation->get_time());
		sim_ty.push_back(body2.get_translation().y());
		step++;
	}

	REQUIRE(sim_t.size() >= 2);

	double max_abs_error = 0.0;
	for (size_t i = 0; i < baseline_t.size(); ++i) {
		const double y_pred = interp_linear(sim_t, sim_ty, baseline_t[i]);
		max_abs_error = std::max(max_abs_error, std::abs(y_pred - baseline_ty[i]));
	}

	CHECK(max_abs_error < 2e-2);
}
