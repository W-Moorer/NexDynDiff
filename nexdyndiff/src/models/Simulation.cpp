#include "Simulation.h"


double nexdyndiff::Simulation::get_time() const
{
	return this->nexdyndiff.current_time;
}

double nexdyndiff::Simulation::get_time_step_size() const
{
	return this->nexdyndiff.dt;
}

int nexdyndiff::Simulation::get_frame() const
{
	return this->nexdyndiff.current_frame;
}

Eigen::Vector3d nexdyndiff::Simulation::get_gravity() const
{
	return this->nexdyndiff.gravity;
}

void nexdyndiff::Simulation::set_gravity(const Eigen::Vector3d& gravity)
{
	this->nexdyndiff.gravity = gravity;
}

nexdyndiff::core::Logger& nexdyndiff::Simulation::get_logger()
{
	return this->nexdyndiff.logger;
}

nexdyndiff::core::Console& nexdyndiff::Simulation::get_console()
{
	return this->nexdyndiff.console;
}

const nexdyndiff::core::Settings& nexdyndiff::Simulation::get_settings() const
{
	return this->nexdyndiff.settings;
}

void nexdyndiff::Simulation::add_time_event(double t0, double t1, std::function<void(double)> action)
{
	this->add_time_event(t0, t1, [action](double t, EventInfo& event_info) { action(t); });
}
void nexdyndiff::Simulation::add_time_event(double t0, double t1, std::function<void(double, EventInfo&)> action)
{
	this->nexdyndiff.script.add_event(
		/* action = */ [action, this](EventInfo& event_info) { action(this->get_time(), event_info); },
		/* run_when = */ [t0, t1, this](EventInfo& event_info) { return this->get_time() >= t0 && this->get_time() < t1; },
		/* delete_when = */ [t1, this](EventInfo& event_info) { return this->get_time() >= t1; }
	);
}

void nexdyndiff::Simulation::run(std::function<void()> callback)
{
	this->run(std::numeric_limits<double>::max(), callback);
}

nexdyndiff::EventDrivenScript& nexdyndiff::Simulation::get_script()
{
	return this->nexdyndiff.script;
}

void nexdyndiff::Simulation::run(double duration, std::function<void()> user_callback)
{
	this->nexdyndiff.run(duration, 
		[user_callback, this]()
		{
			this->nexdyndiff.script.run_a_cycle(this->get_time());
			if (user_callback != nullptr) user_callback();
		}
	);
}

void nexdyndiff::Simulation::run_one_time_step()
{
	this->nexdyndiff.script.run_a_cycle(this->get_time());
	this->nexdyndiff.run_one_step();
}

nexdyndiff::Simulation::Simulation(const core::Settings& settings)
	: nexdyndiff(settings)
{
	// Base dynamics
	spPointDynamics point_dynamics = std::make_shared<PointDynamics>(this->nexdyndiff);
	spRigidBodyDynamics rb_dynamics = std::make_shared<RigidBodyDynamics>(this->nexdyndiff);

	// Physical Systems
	this->deformables = std::make_shared<Deformables>(this->nexdyndiff, point_dynamics);
	this->rigidbodies = std::make_shared<RigidBodies>(this->nexdyndiff, rb_dynamics);

	// Interactions
	this->interactions = std::make_shared<Interactions>(this->nexdyndiff, point_dynamics, rb_dynamics);

	// Presets
	this->presets = std::make_shared<Presets>(this->nexdyndiff, this->deformables, this->rigidbodies, this->interactions);
}
