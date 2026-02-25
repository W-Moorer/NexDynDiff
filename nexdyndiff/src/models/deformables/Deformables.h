#pragma once
#include "DeformablesMeshOutput.h"
#include "DeformablesEnergiesInclude.h"

namespace nexdyndiff
{
	class Deformables
	{
	public:
		/* Methods */
		Deformables(core::NexDynDiff& nexdyndiff, spPointDynamics dyn);

		/* Fields */
		std::shared_ptr<DeformablesMeshOutput> output;

		// Models
		spPointDynamics point_sets;
		std::shared_ptr<EnergyLumpedInertia> lumped_inertia;
		std::shared_ptr<EnergyPrescribedPositions> prescribed_positions;
		std::shared_ptr<EnergySegmentStrain> segment_strain;
		std::shared_ptr<EnergyTriangleStrain> triangle_strain;
		std::shared_ptr<EnergyDiscreteShells> discrete_shells;
		std::shared_ptr<EnergyTetStrain> tet_strain;
	};
}
