#include "Deformables.h"

#include "../../utils/include.h"

nexdyndiff::Deformables::Deformables(core::NexDynDiff& nexdyndiff, spPointDynamics dyn)
	: point_sets(dyn)
{
	this->output = std::make_shared<DeformablesMeshOutput>(nexdyndiff, dyn);
	this->lumped_inertia = std::make_shared<EnergyLumpedInertia>(nexdyndiff, dyn);
	this->prescribed_positions = std::make_shared<EnergyPrescribedPositions>(nexdyndiff, dyn);
	this->segment_strain = std::make_shared<EnergySegmentStrain>(nexdyndiff, dyn);
	this->triangle_strain = std::make_shared<EnergyTriangleStrain>(nexdyndiff, dyn);
	this->discrete_shells = std::make_shared<EnergyDiscreteShells>(nexdyndiff, dyn);
	this->tet_strain = std::make_shared<EnergyTetStrain>(nexdyndiff, dyn);
}

