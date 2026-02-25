#include "Presets.h"

nexdyndiff::Presets::Presets(core::NexDynDiff& nexdyndiff, std::shared_ptr<Deformables> deformables, std::shared_ptr<RigidBodies> rigidbodies, std::shared_ptr<Interactions> interactions)
{
	this->deformables = std::make_shared<DeformablesPresets>(nexdyndiff, deformables, interactions);
	this->rigidbodies = std::make_shared<RigidBodyPresets>(nexdyndiff, rigidbodies, interactions);
}
