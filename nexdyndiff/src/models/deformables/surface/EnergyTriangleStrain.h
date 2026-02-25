#pragma once
#include "../../../core/nexdyndiff.h"
#include "../PointDynamics.h"
#include "../../types.h"

namespace nexdyndiff
{
	class EnergyTriangleStrain
	{
	public:
		/* Types */
		struct Params 
		{
			NEXDYNDIFF_PARAM_ELASTICITY_ONLY()
			NEXDYNDIFF_PARAM_SCALE()
			NEXDYNDIFF_PARAM_NON_NEGATIVE(double, thickness, 0.001)
			NEXDYNDIFF_PARAM_YOUNGS_MODULUS()
			NEXDYNDIFF_PARAM_POISSONS_RATIO()
			NEXDYNDIFF_PARAM_DAMPING()
			NEXDYNDIFF_PARAM_STRAIN_LIMITING()
			NEXDYNDIFF_PARAM_NON_NEGATIVE(double, inflation, 0.0)
		};
		struct Handler { NEXDYNDIFF_COMMON_HANDLER_CONTENTS(EnergyTriangleStrain, Params) };

	private:
		/* Fields */
		const spPointDynamics dyn;
		symx::LabelledConnectivity<5> conn_elasticity_only{ { "idx", "group", "i", "j", "k" } };
		symx::LabelledConnectivity<5> conn_complete{ { "idx", "group", "i", "j", "k" } };

		// Input
		std::vector<bool> elasticity_only;  // per group
		std::vector<double> scale;  // per group
		std::vector<double> thickness;  // group
		std::vector<double> youngs_modulus;  // per group
		std::vector<double> poissons_ratio;  // per group
		std::vector<double> strain_damping;  // per group
		std::vector<double> strain_limit;  // per group
		std::vector<double> strain_limit_stiffness;  // per group
		std::vector<double> inflation;  // per group
		
	public:
		/* Methods */
		EnergyTriangleStrain(nexdyndiff::core::NexDynDiff& nexdyndiff, spPointDynamics dyn);
		Handler add(const PointSetHandler& set, const std::vector<std::array<int, 3>>& triangles, const Params& params);
		Params get_params(const Handler& handler) const;
		void set_params(const Handler& handler, const Params& params);
	};
}
