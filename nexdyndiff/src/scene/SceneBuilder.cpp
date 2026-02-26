#include "SceneBuilder.h"

#include <algorithm>
#include <array>
#include <unordered_set>

#include "../utils/include.h"
#include "SceneMiniParsers.h"

namespace
{
	using nexdyndiff::scene::BuildResult;
	using nexdyndiff::scene::SceneBuilder;
	using nexdyndiff::scene::SceneDescription;
	using nexdyndiff::scene::SceneError;

	void AddError(BuildResult& result, const std::string& path, const std::string& message)
	{
		result.errors.push_back(SceneError{ path, message });
	}

	std::string ToLower(const std::string& value)
	{
		return nexdyndiff::scene::detail::ToLower(value);
	}

	std::filesystem::path ResolvePath(const std::filesystem::path& base_path, const std::filesystem::path& file_path)
	{
		if (file_path.is_absolute() || base_path.empty()) {
			return file_path;
		}
		return base_path / file_path;
	}

	std::string ExtensionLower(const std::filesystem::path& file_path)
	{
		return ToLower(file_path.extension().string());
	}

	void ApplyTransform(nexdyndiff::PointSetHandler& point_set, const nexdyndiff::scene::TransformDefinition& transform)
	{
		if (transform.rotation_deg.x() != 0.0) point_set.add_rotation(transform.rotation_deg.x(), Eigen::Vector3d::UnitX());
		if (transform.rotation_deg.y() != 0.0) point_set.add_rotation(transform.rotation_deg.y(), Eigen::Vector3d::UnitY());
		if (transform.rotation_deg.z() != 0.0) point_set.add_rotation(transform.rotation_deg.z(), Eigen::Vector3d::UnitZ());
		if (transform.translation.norm() != 0.0) point_set.add_displacement(transform.translation);
	}

	void ApplyTransform(nexdyndiff::RigidBodyHandler& rigid_body, const nexdyndiff::scene::TransformDefinition& transform)
	{
		if (transform.rotation_deg.x() != 0.0) rigid_body.add_rotation(transform.rotation_deg.x(), Eigen::Vector3d::UnitX());
		if (transform.rotation_deg.y() != 0.0) rigid_body.add_rotation(transform.rotation_deg.y(), Eigen::Vector3d::UnitY());
		if (transform.rotation_deg.z() != 0.0) rigid_body.add_rotation(transform.rotation_deg.z(), Eigen::Vector3d::UnitZ());
		if (transform.translation.norm() != 0.0) rigid_body.add_translation(transform.translation);
	}

	const nexdyndiff::scene::MaterialDefinition* FindMaterial(
		const SceneDescription& scene,
		const std::string& material_id)
	{
		for (const auto& material : scene.materials) {
			if (material.id == material_id) {
				return &material;
			}
		}
		return nullptr;
	}

	void ApplyMaterialToVolume(const nexdyndiff::scene::MaterialDefinition* material, nexdyndiff::Volume::Params& params)
	{
		if (material == nullptr) return;
		if (material->density.has_value()) params.inertia.density = *material->density;
		if (material->damping.has_value()) params.inertia.damping = *material->damping;
		if (material->youngs_modulus.has_value()) params.strain.youngs_modulus = *material->youngs_modulus;
		if (material->poisson_ratio.has_value()) params.strain.poissons_ratio = *material->poisson_ratio;
		if (material->elasticity_only.has_value()) params.strain.elasticity_only = *material->elasticity_only;
		if (material->strain_limit.has_value()) params.strain.strain_limit = *material->strain_limit;
	}

	void ApplyMaterialToSurface(const nexdyndiff::scene::MaterialDefinition* material, nexdyndiff::Surface::Params& params)
	{
		if (material == nullptr) return;
		if (material->density.has_value()) params.inertia.density = *material->density;
		if (material->damping.has_value()) {
			params.inertia.damping = *material->damping;
			params.strain.damping = *material->damping;
			params.bending.damping = *material->damping;
		}
		if (material->youngs_modulus.has_value()) params.strain.youngs_modulus = *material->youngs_modulus;
		if (material->poisson_ratio.has_value()) params.strain.poissons_ratio = *material->poisson_ratio;
		if (material->elasticity_only.has_value()) {
			params.strain.elasticity_only = *material->elasticity_only;
			params.bending.elasticity_only = *material->elasticity_only;
		}
		if (material->strain_limit.has_value()) params.strain.strain_limit = *material->strain_limit;
		if (material->bending_stiffness.has_value()) params.bending.stiffness = *material->bending_stiffness;
	}

	void ApplyMaterialToLine(const nexdyndiff::scene::MaterialDefinition* material, nexdyndiff::Line::Params& params)
	{
		if (material == nullptr) return;
		if (material->density.has_value()) params.inertia.density = *material->density;
		if (material->damping.has_value()) {
			params.inertia.damping = *material->damping;
			params.strain.damping = *material->damping;
		}
		if (material->youngs_modulus.has_value()) params.strain.youngs_modulus = *material->youngs_modulus;
		if (material->section_radius.has_value()) params.strain.section_radius = *material->section_radius;
		if (material->elasticity_only.has_value()) params.strain.elasticity_only = *material->elasticity_only;
		if (material->strain_limit.has_value()) params.strain.strain_limit = *material->strain_limit;
	}

	bool EnsureContactThicknessIfNeeded(
		BuildResult& result,
		const std::string& object_path,
		bool contact_initialized,
		bool has_global_default_thickness,
		const std::optional<double>& explicit_contact_thickness)
	{
		if (!contact_initialized) {
			return true;
		}
		if (has_global_default_thickness || explicit_contact_thickness.has_value()) {
			return true;
		}
		AddError(result, object_path, "Missing contact thickness. Set object/material thickness or global default_contact_thickness.");
		return false;
	}

	template<std::size_t N>
	bool LoadVtkMesh(
		const std::filesystem::path& full_path,
		nexdyndiff::Mesh<N>& out_mesh,
		BuildResult& result,
		const std::string& path)
	{
		if (!std::filesystem::exists(full_path)) {
			AddError(result, path, "Mesh file not found: " + full_path.string());
			return false;
		}
		out_mesh = nexdyndiff::load_vtk<N>(full_path.string());
		return true;
	}
}

nexdyndiff::scene::SceneBuilder::SceneBuilder(
	const SceneDescription& description,
	const std::filesystem::path& base_path)
	: m_description(description), m_base_path(base_path)
{
}

nexdyndiff::scene::BuildResult nexdyndiff::scene::SceneBuilder::Build(Simulation& simulation) const
{
	BuildResult result;
	std::unordered_set<std::string> disabled_contacts;

	ContactGlobalParams global_contact_params = simulation.interactions->contact->get_global_params();
	if (m_description.contact_global.default_contact_thickness.has_value()) global_contact_params.default_contact_thickness = *m_description.contact_global.default_contact_thickness;
	if (m_description.contact_global.min_contact_stiffness.has_value()) global_contact_params.min_contact_stiffness = *m_description.contact_global.min_contact_stiffness;
	if (m_description.contact_global.friction_stick_slide_threshold.has_value()) global_contact_params.friction_stick_slide_threshold = *m_description.contact_global.friction_stick_slide_threshold;
	simulation.interactions->contact->set_global_params(global_contact_params);

	const bool has_global_default_thickness = global_contact_params.default_contact_thickness > 0.0;
	const bool contact_initialized = simulation.get_settings().simulation.init_frictional_contact;

	for (const auto& deformable : m_description.deformables) {
		const std::string path_prefix = "objects.deformables." + deformable.id;
		const auto* material = deformable.material_id.empty() ? nullptr : FindMaterial(m_description, deformable.material_id);
		if (!deformable.material_id.empty() && material == nullptr) {
			AddError(result, path_prefix + ".material", "Unknown material: " + deformable.material_id);
			continue;
		}

		ContactParams contact_params;
		std::optional<double> contact_thickness = deformable.contact_thickness;
		if (!contact_thickness.has_value() && material != nullptr) {
			contact_thickness = material->contact_thickness;
		}
		if (contact_thickness.has_value() && *contact_thickness <= 0.0) {
			AddError(result, path_prefix + ".contact.thickness", "Contact thickness must be positive");
			continue;
		}
		if (!EnsureContactThicknessIfNeeded(result, path_prefix + ".contact", contact_initialized, has_global_default_thickness, contact_thickness)) {
			continue;
		}
		if (contact_thickness.has_value()) {
			contact_params.contact_thickness = *contact_thickness;
		}

		const std::string type = ToLower(deformable.type);
		if (type == "volume") {
			Volume::Params params = Volume::Params::Soft_Rubber();
			params.contact = contact_params;
			ApplyMaterialToVolume(material, params);

			nexdyndiff::Mesh<4> mesh;
			if (ToLower(deformable.geometry.type) == "file") {
				const std::filesystem::path full_path = ResolvePath(m_base_path, deformable.geometry.path);
				if (!LoadVtkMesh<4>(full_path, mesh, result, path_prefix + ".geometry.path")) continue;
			}
			else if (ToLower(deformable.geometry.type) == "grid" || ToLower(deformable.geometry.generator) == "tet_grid") {
				mesh = nexdyndiff::generate_tet_grid(deformable.geometry.center3, deformable.geometry.size3, deformable.geometry.subdivisions3);
			}
			else {
				AddError(result, path_prefix + ".geometry.type", "Unsupported volume geometry type: " + deformable.geometry.type);
				continue;
			}

			auto handler = simulation.presets->deformables->add_volume(deformable.label, mesh.vertices, mesh.conn, params);
			ApplyTransform(handler.point_set, deformable.transform);
			result.point_sets.insert_or_assign(deformable.id, handler.point_set);
			result.contacts.insert_or_assign(deformable.id, handler.contact);
		}
		else if (type == "surface") {
			Surface::Params params = Surface::Params::Cotton_Fabric();
			params.contact = contact_params;
			ApplyMaterialToSurface(material, params);

			nexdyndiff::Mesh<3> mesh;
			if (ToLower(deformable.geometry.type) == "file") {
				const std::filesystem::path full_path = ResolvePath(m_base_path, deformable.geometry.path);
				const std::string extension = ExtensionLower(full_path);
				if (extension == ".vtk") {
					if (!LoadVtkMesh<3>(full_path, mesh, result, path_prefix + ".geometry.path")) continue;
				}
				else if (extension == ".obj") {
					if (!std::filesystem::exists(full_path)) {
						AddError(result, path_prefix + ".geometry.path", "Mesh file not found: " + full_path.string());
						continue;
					}
					const std::vector<nexdyndiff::Mesh<3>> meshes = nexdyndiff::load_obj(full_path.string());
					if (meshes.empty()) {
						AddError(result, path_prefix + ".geometry.path", "OBJ file does not contain any mesh: " + full_path.string());
						continue;
					}
					mesh = meshes.front();
				}
				else {
					AddError(result, path_prefix + ".geometry.path", "Unsupported surface file extension: " + extension);
					continue;
				}
			}
			else if (ToLower(deformable.geometry.type) == "grid" || ToLower(deformable.geometry.generator) == "triangle_grid") {
				mesh = nexdyndiff::generate_triangle_grid(deformable.geometry.center2, deformable.geometry.size2, deformable.geometry.subdivisions2);
			}
			else {
				AddError(result, path_prefix + ".geometry.type", "Unsupported surface geometry type: " + deformable.geometry.type);
				continue;
			}

			auto handler = simulation.presets->deformables->add_surface(deformable.label, mesh.vertices, mesh.conn, params);
			ApplyTransform(handler.point_set, deformable.transform);
			result.point_sets.insert_or_assign(deformable.id, handler.point_set);
			result.contacts.insert_or_assign(deformable.id, handler.contact);
		}
		else if (type == "line") {
			Line::Params params = Line::Params::Elastic_Rubberband();
			params.contact = contact_params;
			ApplyMaterialToLine(material, params);

			nexdyndiff::Mesh<2> mesh;
			if (ToLower(deformable.geometry.type) == "file") {
				const std::filesystem::path full_path = ResolvePath(m_base_path, deformable.geometry.path);
				if (!LoadVtkMesh<2>(full_path, mesh, result, path_prefix + ".geometry.path")) continue;
			}
			else if (ToLower(deformable.geometry.type) == "grid" || ToLower(deformable.geometry.generator) == "segment_line") {
				mesh = nexdyndiff::generate_segment_line(deformable.geometry.begin, deformable.geometry.end, deformable.geometry.n_segments);
			}
			else {
				AddError(result, path_prefix + ".geometry.type", "Unsupported line geometry type: " + deformable.geometry.type);
				continue;
			}

			auto handler = simulation.presets->deformables->add_line(deformable.label, mesh.vertices, mesh.conn, params);
			ApplyTransform(handler.point_set, deformable.transform);
			result.point_sets.insert_or_assign(deformable.id, handler.point_set);
			result.contacts.insert_or_assign(deformable.id, handler.contact);
		}
		else {
			AddError(result, path_prefix + ".type", "Unsupported deformable type: " + deformable.type);
			continue;
		}

		if (!deformable.contact_enabled) {
			disabled_contacts.insert(deformable.id);
		}
	}

	for (const auto& rigid_body : m_description.rigid_bodies) {
		const std::string path_prefix = "objects.rigidbodies." + rigid_body.id;

		ContactParams contact_params;
		if (rigid_body.contact_thickness.has_value() && *rigid_body.contact_thickness <= 0.0) {
			AddError(result, path_prefix + ".contact.thickness", "Contact thickness must be positive");
			continue;
		}
		if (!EnsureContactThicknessIfNeeded(
			result,
			path_prefix + ".contact",
			contact_initialized,
			has_global_default_thickness,
			rigid_body.contact_thickness)) {
			continue;
		}
		if (rigid_body.contact_thickness.has_value()) {
			contact_params.contact_thickness = *rigid_body.contact_thickness;
		}

		const std::string geometry_type = ToLower(rigid_body.geometry.type);
		RigidBody::Handler rigid_body_handler;
		if (geometry_type == "box") {
			auto added = simulation.presets->rigidbodies->add_box(rigid_body.label, rigid_body.mass, rigid_body.geometry.size3, contact_params);
			rigid_body_handler = added.handler;
		}
		else if (geometry_type == "sphere") {
			auto added = simulation.presets->rigidbodies->add_sphere(rigid_body.label, rigid_body.mass, rigid_body.geometry.radius, rigid_body.geometry.subdivisions, contact_params);
			rigid_body_handler = added.handler;
		}
		else if (geometry_type == "cylinder") {
			auto added = simulation.presets->rigidbodies->add_cylinder(
				rigid_body.label,
				rigid_body.mass,
				rigid_body.geometry.radius,
				rigid_body.geometry.height,
				rigid_body.geometry.slices,
				rigid_body.geometry.stacks,
				contact_params);
			rigid_body_handler = added.handler;
		}
		else if (geometry_type == "torus") {
			auto added = simulation.presets->rigidbodies->add_torus(
				rigid_body.label,
				rigid_body.mass,
				rigid_body.geometry.outer_radius,
				rigid_body.geometry.inner_radius,
				rigid_body.geometry.slices,
				rigid_body.geometry.stacks,
				contact_params);
			rigid_body_handler = added.handler;
		}
		else {
			AddError(result, path_prefix + ".geometry.type", "Unsupported rigid body geometry type: " + rigid_body.geometry.type);
			continue;
		}

		ApplyTransform(rigid_body_handler.rigidbody, rigid_body.transform);
		if (rigid_body.inertia_tensor.has_value()) {
			rigid_body_handler.rigidbody.set_local_inertia_tensor(*rigid_body.inertia_tensor);
		}
		if (rigid_body.fixed) {
			simulation.rigidbodies->add_constraint_fix(rigid_body_handler.rigidbody);
		}

		result.rigid_bodies.insert_or_assign(rigid_body.id, rigid_body_handler.rigidbody);
		result.contacts.insert_or_assign(rigid_body.id, rigid_body_handler.contact);
	}

	size_t prescribed_bc_counter = 0;
	for (const auto& bc : m_description.boundary_conditions) {
		const std::string type = ToLower(bc.type);
		if (type == "prescribed_position") {
			auto it = result.point_sets.find(bc.target);
			if (it == result.point_sets.end()) {
				AddError(result, "boundary_conditions", "Unknown prescribed_position target: " + bc.target);
				continue;
			}

			EnergyPrescribedPositions::Params params;
			params.stiffness = bc.stiffness;
			params.tolerance = bc.tolerance;

			const std::string selector_type = ToLower(bc.selector.type);
			EnergyPrescribedPositions::Handler bc_handler;
			if (selector_type == "inside_aabb") {
				bc_handler = simulation.deformables->prescribed_positions->add_inside_aabb(it->second, bc.selector.center, bc.selector.halfsize, params);
			}
			else if (selector_type == "outside_aabb") {
				bc_handler = simulation.deformables->prescribed_positions->add_outside_aabb(it->second, bc.selector.center, bc.selector.halfsize, params);
			}
			else {
				AddError(result, "boundary_conditions", "Unsupported selector type: " + bc.selector.type);
			}

			if (bc_handler.is_valid()) {
				const std::string bc_id = bc.id.empty()
					? (bc.target + "_bc_" + std::to_string(prescribed_bc_counter++))
					: bc.id;
				if (result.prescribed_positions.find(bc_id) != result.prescribed_positions.end()) {
					AddError(result, "boundary_conditions." + bc_id, "Duplicate prescribed position boundary condition id");
				}
				else {
					result.prescribed_positions.insert_or_assign(bc_id, bc_handler);
				}
			}
		}
		else if (type == "rigidbody_constraint") {
			const std::string constraint_type = bc.constraint_type.empty() ? "fix" : ToLower(bc.constraint_type);

			auto it_target = result.rigid_bodies.find(bc.target);
			auto it_body_a = result.rigid_bodies.find(bc.body_a);
			auto it_body_b = result.rigid_bodies.find(bc.body_b);

			if (constraint_type == "fix") {
				if (it_target == result.rigid_bodies.end()) {
					AddError(result, "boundary_conditions", "Unknown rigidbody_constraint target: " + bc.target);
					continue;
				}
				simulation.rigidbodies->add_constraint_fix(it_target->second);
			}
			else if (constraint_type == "point") {
				if (it_body_a == result.rigid_bodies.end() || it_body_b == result.rigid_bodies.end()) {
					AddError(result, "boundary_conditions", "Unknown body_a or body_b for point constraint");
					continue;
				}
				simulation.rigidbodies->add_constraint_point(it_body_a->second, it_body_b->second, bc.pivot);
			}
			else if (constraint_type == "point_on_axis") {
				if (it_body_a == result.rigid_bodies.end() || it_body_b == result.rigid_bodies.end()) {
					AddError(result, "boundary_conditions", "Unknown body_a or body_b for point_on_axis constraint");
					continue;
				}
				simulation.rigidbodies->add_constraint_point_on_axis(it_body_a->second, it_body_b->second, bc.pivot, bc.axis);
			}
			else if (constraint_type == "direction") {
				if (it_body_a == result.rigid_bodies.end() || it_body_b == result.rigid_bodies.end()) {
					AddError(result, "boundary_conditions", "Unknown body_a or body_b for direction constraint");
					continue;
				}
				simulation.rigidbodies->add_constraint_direction(it_body_a->second, it_body_b->second, bc.axis);
			}
			else if (constraint_type == "distance") {
				if (it_body_a == result.rigid_bodies.end() || it_body_b == result.rigid_bodies.end()) {
					AddError(result, "boundary_conditions", "Unknown body_a or body_b for distance constraint");
					continue;
				}
				simulation.rigidbodies->add_constraint_distance(it_body_a->second, it_body_b->second, bc.pivot, bc.pivot + bc.axis * bc.target_distance);
			}
			else if (constraint_type == "distance_limits") {
				if (it_body_a == result.rigid_bodies.end() || it_body_b == result.rigid_bodies.end()) {
					AddError(result, "boundary_conditions", "Unknown body_a or body_b for distance_limits constraint");
					continue;
				}
				simulation.rigidbodies->add_constraint_distance_limits(it_body_a->second, it_body_b->second, bc.pivot, bc.pivot + bc.axis, bc.min_limit, bc.max_limit);
			}
			else if (constraint_type == "angle_limit") {
				if (it_body_a == result.rigid_bodies.end() || it_body_b == result.rigid_bodies.end()) {
					AddError(result, "boundary_conditions", "Unknown body_a or body_b for angle_limit constraint");
					continue;
				}
				simulation.rigidbodies->add_constraint_angle_limit(it_body_a->second, it_body_b->second, bc.axis, bc.admissible_angle_deg);
			}
			else if (constraint_type == "spring") {
				if (it_body_a == result.rigid_bodies.end() || it_body_b == result.rigid_bodies.end()) {
					AddError(result, "boundary_conditions", "Unknown body_a or body_b for spring constraint");
					continue;
				}
				simulation.rigidbodies->add_constraint_spring(it_body_a->second, it_body_b->second, bc.pivot, bc.pivot + bc.axis, bc.stiffness, bc.damping);
			}
			else if (constraint_type == "attachment") {
				if (it_body_a == result.rigid_bodies.end() || it_body_b == result.rigid_bodies.end()) {
					AddError(result, "boundary_conditions", "Unknown body_a or body_b for attachment constraint");
					continue;
				}
				simulation.rigidbodies->add_constraint_attachment(it_body_a->second, it_body_b->second);
			}
			else if (constraint_type == "point_with_angle_limit") {
				if (it_body_a == result.rigid_bodies.end() || it_body_b == result.rigid_bodies.end()) {
					AddError(result, "boundary_conditions", "Unknown body_a or body_b for point_with_angle_limit constraint");
					continue;
				}
				simulation.rigidbodies->add_constraint_point_with_angle_limit(it_body_a->second, it_body_b->second, bc.pivot, bc.axis, bc.admissible_angle_deg);
			}
			else if (constraint_type == "hinge") {
				if (it_body_a == result.rigid_bodies.end() || it_body_b == result.rigid_bodies.end()) {
					AddError(result, "boundary_conditions", "Unknown body_a or body_b for hinge constraint");
					continue;
				}
				simulation.rigidbodies->add_constraint_hinge(it_body_a->second, it_body_b->second, bc.pivot, bc.axis);
			}
			else if (constraint_type == "hinge_with_limits") {
				if (it_body_a == result.rigid_bodies.end() || it_body_b == result.rigid_bodies.end()) {
					AddError(result, "boundary_conditions", "Unknown body_a or body_b for hinge_with_limits constraint");
					continue;
				}
				simulation.rigidbodies->add_constraint_hinge_with_angle_limit(it_body_a->second, it_body_b->second, bc.pivot, bc.axis, bc.admissible_angle_deg);
			}
			else if (constraint_type == "spring_with_limits") {
				if (it_body_a == result.rigid_bodies.end() || it_body_b == result.rigid_bodies.end()) {
					AddError(result, "boundary_conditions", "Unknown body_a or body_b for spring_with_limits constraint");
					continue;
				}
				simulation.rigidbodies->add_constraint_spring_with_limits(it_body_a->second, it_body_b->second, bc.pivot, bc.pivot + bc.axis, bc.stiffness, bc.min_limit, bc.max_limit, bc.damping);
			}
			else if (constraint_type == "slider") {
				if (it_body_a == result.rigid_bodies.end() || it_body_b == result.rigid_bodies.end()) {
					AddError(result, "boundary_conditions", "Unknown body_a or body_b for slider constraint");
					continue;
				}
				simulation.rigidbodies->add_constraint_slider(it_body_a->second, it_body_b->second, bc.pivot, bc.axis);
			}
			else if (constraint_type == "prismatic_slider") {
				if (it_body_a == result.rigid_bodies.end() || it_body_b == result.rigid_bodies.end()) {
					AddError(result, "boundary_conditions", "Unknown body_a or body_b for prismatic_slider constraint");
					continue;
				}
				simulation.rigidbodies->add_constraint_prismatic_slider(it_body_a->second, it_body_b->second, bc.pivot, bc.axis);
			}
			else if (constraint_type == "prismatic_press") {
				if (it_body_a == result.rigid_bodies.end() || it_body_b == result.rigid_bodies.end()) {
					AddError(result, "boundary_conditions", "Unknown body_a or body_b for prismatic_press constraint");
					continue;
				}
				simulation.rigidbodies->add_constraint_prismatic_press(it_body_a->second, it_body_b->second, bc.pivot, bc.axis, bc.target_velocity, bc.max_force);
			}
			else if (constraint_type == "motor") {
				if (it_body_a == result.rigid_bodies.end() || it_body_b == result.rigid_bodies.end()) {
					AddError(result, "boundary_conditions", "Unknown body_a or body_b for motor constraint");
					continue;
				}
				simulation.rigidbodies->add_constraint_motor(it_body_a->second, it_body_b->second, bc.pivot, bc.axis, bc.target_velocity, bc.max_force);
			}
			else {
				AddError(result, "boundary_conditions", "Unsupported rigidbody constraint type: " + bc.constraint_type);
			}
		}
		else {
			AddError(result, "boundary_conditions", "Unsupported boundary condition type: " + bc.type);
		}
	}

	for (const auto& event : m_description.scripted_events) {
		const std::string event_type = ToLower(event.type);
		if (event_type != "time_event") {
			AddError(result, "scripted_events", "Unsupported event type: " + event.type);
			continue;
		}
		if (event.end_time <= event.start_time) {
			AddError(result, "scripted_events", "Invalid event time range for target: " + event.target);
			continue;
		}

		const std::string action_type = ToLower(event.action.type);
		if (action_type == "set_gravity") {
			const Eigen::Vector3d target_gravity = event.action.gravity;
			const bool linear_blend = ToLower(event.action.blend) == "linear";
			const double t0 = event.start_time;
			const double t1 = event.end_time;
			simulation.add_time_event(t0, t1,
				[&simulation, t0, t1, target_gravity, linear_blend](double t, EventInfo& event_info)
				{
					if (!linear_blend) {
						simulation.set_gravity(target_gravity);
						return;
					}

					if (!event_info.has_data()) {
						const Eigen::Vector3d gravity = simulation.get_gravity();
						std::array<double, 3> data = { gravity.x(), gravity.y(), gravity.z() };
						event_info.pack(data);
					}

					std::array<double, 3> g0_data = event_info.unpack<std::array<double, 3>>();
					const Eigen::Vector3d g0(g0_data[0], g0_data[1], g0_data[2]);
					const double alpha = std::clamp((t - t0) / std::max(1e-12, t1 - t0), 0.0, 1.0);
					simulation.set_gravity((1.0 - alpha) * g0 + alpha * target_gravity);
				});
		}
		else if (action_type == "transform_bc") {
			auto it = result.prescribed_positions.find(event.target);
			if (it == result.prescribed_positions.end()) {
				AddError(result, "scripted_events", "Unknown prescribed boundary condition target: " + event.target);
				continue;
			}

			auto bc_handler = it->second;
			const double angular_velocity = event.action.angular_velocity;
			const Eigen::Vector3d axis = event.action.axis;
			const Eigen::Vector3d translation = event.action.translation;
			const Eigen::Vector3d translation_velocity = event.action.translation_velocity;
			const double t0 = event.start_time;
			simulation.add_time_event(t0, event.end_time,
				[bc_handler, angular_velocity, axis, translation, translation_velocity, t0](double t, EventInfo&) mutable
				{
					const double dt = std::max(0.0, t - t0);
					const Eigen::Vector3d t_current = translation + dt * translation_velocity;
					const double angle = angular_velocity * dt;
					bc_handler.set_transformation(t_current, angle, axis);
				});
		}
		else {
			AddError(result, "scripted_events", "Unsupported action type: " + event.action.type);
		}
	}

	for (const auto& pair : m_description.contact_pairs) {
		auto it0 = result.contacts.find(pair.object1);
		auto it1 = result.contacts.find(pair.object2);
		if (it0 == result.contacts.end() || it1 == result.contacts.end()) {
			AddError(result, "interactions.contact.pairs", "Unknown contact pair object(s): " + pair.object1 + ", " + pair.object2);
			continue;
		}
		if (!pair.enabled) {
			it0->second.disable_collision(it1->second);
			continue;
		}
		if (pair.friction.has_value()) {
			it0->second.set_friction(it1->second, *pair.friction);
		}
	}

	for (const auto& pair : m_description.disabled_contact_pairs) {
		auto it0 = result.contacts.find(pair.object1);
		auto it1 = result.contacts.find(pair.object2);
		if (it0 == result.contacts.end() || it1 == result.contacts.end()) {
			AddError(result, "interactions.contact.disabled_pairs", "Unknown contact pair object(s): " + pair.object1 + ", " + pair.object2);
			continue;
		}
		it0->second.disable_collision(it1->second);
	}

	if (!disabled_contacts.empty()) {
		for (const auto& first_id : disabled_contacts) {
			auto it_first = result.contacts.find(first_id);
			if (it_first == result.contacts.end()) continue;
			for (const auto& kv : result.contacts) {
				if (kv.first == first_id) continue;
				it_first->second.disable_collision(kv.second);
			}
		}
	}

	result.success = result.errors.empty();
	return result;
}
