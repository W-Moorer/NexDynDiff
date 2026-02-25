#include "PointDynamics.h"

#include "../time_integration.h"

nexdyndiff::PointDynamics::PointDynamics(nexdyndiff::core::NexDynDiff& nexdyndiff)
{
	this->dof = nexdyndiff.global_energy.add_dof_array(this->v1.data, "PointDynamics.v1");
	nexdyndiff.callbacks.add_before_time_step([&]() { this->_before_time_step(nexdyndiff); });
	nexdyndiff.callbacks.add_on_time_step_accepted([&]() { this->_on_time_step_accepted(nexdyndiff); });
}

nexdyndiff::PointSetHandler nexdyndiff::PointDynamics::add(const std::vector<Eigen::Vector3d>& x, const std::string& label)
{
	std::vector<Eigen::Vector3d> zero(x.size(), Eigen::Vector3d::Zero());

	const int glob_idx = this->X.append(x);
	this->x0.append(x);
	this->x1.append(x);
	this->v0.append(zero);
	this->v1.append(zero);
	this->a.append(zero);
	this->f.append(zero);
	this->labels.push_back(label.empty() ? "point_set_" + std::to_string(glob_idx) : label);

	return PointSetHandler(this, glob_idx);
}

int nexdyndiff::PointDynamics::size() const
{
	return this->X.size();
}

int nexdyndiff::PointDynamics::get_set_size(int idx_in_ps) const
{
	return this->X.get_set_size(idx_in_ps);
}

int nexdyndiff::PointDynamics::get_begin(int idx_in_ps) const
{
	return this->X.get_begin(idx_in_ps);
}

int nexdyndiff::PointDynamics::get_end(int idx_in_ps) const
{
	return this->X.get_end(idx_in_ps);
}

int nexdyndiff::PointDynamics::get_global_index(int idx_in_ps, int local_index) const
{
	return this->X.get_global_index(idx_in_ps, local_index);
}

Eigen::Vector3d nexdyndiff::PointDynamics::get_x1(int global_index, double dt) const
{
	return time_integration(this->x0[global_index], this->v1[global_index], dt);
}

void nexdyndiff::PointDynamics::_before_time_step(nexdyndiff::core::NexDynDiff& nexdyndiff)
{
	// Set next time velocities estimation to zero to avoid invalid state outside of the minimzer
	std::fill(this->v1.data.begin(), this->v1.data.end(), Eigen::Vector3d::Zero());
}

void nexdyndiff::PointDynamics::_on_time_step_accepted(nexdyndiff::core::NexDynDiff& nexdyndiff)
{
	// Set final positions with solved velocities
	const double dt = nexdyndiff.dt;
	for (int i = 0; i < this->size(); i++) {
		this->x1[i] = time_integration(this->x0[i], this->v1[i], dt);
	}

	// x0 <- x1
	this->x0 = this->x1;
	this->v0 = this->v1;
}
