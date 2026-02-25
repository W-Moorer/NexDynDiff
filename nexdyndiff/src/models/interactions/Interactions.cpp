#include "Interactions.h"

using namespace nexdyndiff;

nexdyndiff::Interactions::Interactions(core::NexDynDiff& nexdyndiff, spPointDynamics dyn, spRigidBodyDynamics rb)
	: dyn(dyn), rb(rb)
{
	this->attachments = std::make_shared<EnergyAttachments>(nexdyndiff, dyn, rb);
	this->contact = std::make_shared<EnergyFrictionalContact>(nexdyndiff, dyn, rb);
}
