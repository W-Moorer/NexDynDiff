#include <cmath>
#include <vector>

#include "nexdyndiff.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using namespace Catch::Matchers;

TEST_CASE("DifferentiateTimeSeries returns empty on size mismatch", "[time_series_utils]")
{
	const std::vector<double> time{ 0.0, 0.1, 0.2 };
	const std::vector<double> values{ 1.0, 2.0 };

	const auto derivative = nexdyndiff::DifferentiateTimeSeries(time, values);
	CHECK(derivative.empty());
}

TEST_CASE("DifferentiateTimeSeries computes scalar derivative for linear signal", "[time_series_utils]")
{
	const std::vector<double> time{ 0.0, 0.1, 0.3, 0.6 };
	std::vector<double> values;
	values.reserve(time.size());
	for (const double t : time) {
		values.push_back(2.0 + 4.0 * t);
	}

	const auto derivative = nexdyndiff::DifferentiateTimeSeries(time, values);
	REQUIRE(derivative.size() == time.size());
	for (const double d : derivative) {
		CHECK_THAT(d, WithinAbs(4.0, 1e-12));
	}
}

TEST_CASE("DifferentiateTimeSeries computes vector derivative for linear signal", "[time_series_utils]")
{
	const std::vector<double> time{ 0.0, 0.2, 0.5, 0.9 };
	std::vector<Eigen::Vector3d> values;
	values.reserve(time.size());
	for (const double t : time) {
		values.push_back(Eigen::Vector3d(1.0 + 3.0 * t, -2.0 + 0.5 * t, 7.0 - 2.0 * t));
	}

	const auto derivative = nexdyndiff::DifferentiateTimeSeries(time, values);
	REQUIRE(derivative.size() == time.size());
	for (const auto& d : derivative) {
		CHECK_THAT(d.x(), WithinAbs(3.0, 1e-12));
		CHECK_THAT(d.y(), WithinAbs(0.5, 1e-12));
		CHECK_THAT(d.z(), WithinAbs(-2.0, 1e-12));
	}
}

TEST_CASE("DifferentiateTimeSeries handles repeated timestamps with min_dt guard", "[time_series_utils]")
{
	const std::vector<double> time{ 0.0, 0.0, 1.0 };
	const std::vector<double> values{ 0.0, 1.0, 2.0 };

	const auto derivative = nexdyndiff::DifferentiateTimeSeries(time, values);
	REQUIRE(derivative.size() == time.size());
	for (const double d : derivative) {
		CHECK(std::isfinite(d));
	}
	CHECK_THAT(derivative[1], WithinAbs(2.0, 1e-12));
}
