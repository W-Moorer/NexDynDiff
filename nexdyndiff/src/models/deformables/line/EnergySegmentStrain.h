#pragma once
#include "../../../core/nexdyndiff.h"
#include "../PointDynamics.h"
#include "../../types.h"
#include "../../types.h"

namespace nexdyndiff
{
	class EnergySegmentStrain
	{
	public:
		/* Types */
		struct Params { 
			NEXDYNDIFF_PARAM_ELASTICITY_ONLY()
			NEXDYNDIFF_PARAM_SCALE()
			NEXDYNDIFF_PARAM_NON_NEGATIVE(double, section_radius, 5e-3)
			NEXDYNDIFF_PARAM_YOUNGS_MODULUS()
			NEXDYNDIFF_PARAM_DAMPING()
			NEXDYNDIFF_PARAM_STRAIN_LIMITING()
		};
		struct Handler { NEXDYNDIFF_COMMON_HANDLER_CONTENTS(EnergySegmentStrain, Params) };

	private:
		/* Fields */
		const spPointDynamics dyn;
		symx::LabelledConnectivity<4> conn_elasticity_only{ { "idx", "group", "i", "j" } };
		symx::LabelledConnectivity<4> conn_complete{ { "idx", "group", "i", "j" } };

		std::vector<bool> elasticity_only;  // per group
		std::vector<double> scale;  // per group
		std::vector<double> section_radius;  // group
		std::vector<double> youngs_modulus;  // per group
		std::vector<double> strain_damping;  // per group
		std::vector<double> strain_limit;  // per group
		std::vector<double> strain_limit_stiffness;  // per group

	public:
		/* Methods */
		EnergySegmentStrain(core::NexDynDiff& nexdyndiff, spPointDynamics dyn);
		Handler add(const PointSetHandler& set, const std::vector<std::array<int, 2>>& segments, const Params& params);
		Params get_params(const Handler& handler) const;
		void set_params(const Handler& handler, const Params& params);
	};
}
