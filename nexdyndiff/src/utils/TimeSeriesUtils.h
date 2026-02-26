#pragma once

#include <cmath>
#include <vector>

namespace nexdyndiff
{
	template <typename TValue>
	std::vector<TValue> DifferentiateTimeSeries(
		const std::vector<double>& time,
		const std::vector<TValue>& values,
		double min_dt = 1e-12)
	{
		if (time.size() != values.size()) {
			return {};
		}

		std::vector<TValue> derivative;
		if (values.empty()) {
			return derivative;
		}

		derivative.assign(values.size(), values.front() * 0.0);
		if (values.size() == 1) {
			return derivative;
		}

		auto safe_dt = [min_dt](double dt)
			{
				if (std::abs(dt) < min_dt) {
					return (dt < 0.0) ? -min_dt : min_dt;
				}
				return dt;
			};

		derivative[0] = (values[1] - values[0]) / safe_dt(time[1] - time[0]);
		for (size_t i = 1; i + 1 < values.size(); ++i) {
			derivative[i] = (values[i + 1] - values[i - 1]) / safe_dt(time[i + 1] - time[i - 1]);
		}
		const size_t last = values.size() - 1;
		derivative[last] = (values[last] - values[last - 1]) / safe_dt(time[last] - time[last - 1]);

		return derivative;
	}
}

