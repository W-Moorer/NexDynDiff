#include "SceneParser.h"

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_set>

#include "SceneMiniParsers.h"

namespace
{
	using nexdyndiff::scene::ParseResult;
	using nexdyndiff::scene::SceneDescription;
	using nexdyndiff::scene::SceneError;
	using namespace nexdyndiff::scene::detail;

	void AddError(ParseResult& result, const std::string& path, const std::string& message)
	{
		result.errors.push_back(SceneError{ path, message });
	}

	const JsonValue* JGet(const JsonValue* object, const std::string& key)
	{
		if (object == nullptr || !object->IsObject()) return nullptr;
		return object->Find(key);
	}

	std::optional<std::string> JString(const JsonValue* value)
	{
		if (value == nullptr || !value->IsString()) return std::nullopt;
		return value->s;
	}

	std::optional<double> JDouble(const JsonValue* value)
	{
		if (value == nullptr || !value->IsNumber()) return std::nullopt;
		return value->n;
	}

	std::optional<int> JInt(const JsonValue* value)
	{
		if (value == nullptr || !value->IsNumber()) return std::nullopt;
		const double v = value->n;
		if (std::floor(v) != v) return std::nullopt;
		if (v < (double)std::numeric_limits<int>::min() || v > (double)std::numeric_limits<int>::max()) return std::nullopt;
		return (int)v;
	}

	std::optional<bool> JBool(const JsonValue* value)
	{
		if (value == nullptr || !value->IsBool()) return std::nullopt;
		return value->b;
	}

	bool JVec2(const JsonValue* value, Eigen::Vector2d& out)
	{
		if (value == nullptr || !value->IsArray() || value->a.size() != 2) return false;
		auto x = JDouble(&value->a[0]);
		auto y = JDouble(&value->a[1]);
		if (!x.has_value() || !y.has_value()) return false;
		out = { *x, *y };
		return true;
	}

	bool JVec3(const JsonValue* value, Eigen::Vector3d& out)
	{
		if (value == nullptr || !value->IsArray() || value->a.size() != 3) return false;
		auto x = JDouble(&value->a[0]);
		auto y = JDouble(&value->a[1]);
		auto z = JDouble(&value->a[2]);
		if (!x.has_value() || !y.has_value() || !z.has_value()) return false;
		out = { *x, *y, *z };
		return true;
	}

	bool JInt2(const JsonValue* value, std::array<int, 2>& out)
	{
		if (value == nullptr || !value->IsArray() || value->a.size() != 2) return false;
		auto x = JInt(&value->a[0]);
		auto y = JInt(&value->a[1]);
		if (!x.has_value() || !y.has_value()) return false;
		out = { *x, *y };
		return true;
	}

	bool JInt3(const JsonValue* value, std::array<int, 3>& out)
	{
		if (value == nullptr || !value->IsArray() || value->a.size() != 3) return false;
		auto x = JInt(&value->a[0]);
		auto y = JInt(&value->a[1]);
		auto z = JInt(&value->a[2]);
		if (!x.has_value() || !y.has_value() || !z.has_value()) return false;
		out = { *x, *y, *z };
		return true;
	}

	bool JMat3(const JsonValue* value, Eigen::Matrix3d& out)
	{
		if (value == nullptr || !value->IsArray()) return false;

		if (value->a.size() == 9) {
			std::array<double, 9> m = {};
			for (int i = 0; i < 9; ++i) {
				auto v = JDouble(&value->a[(size_t)i]);
				if (!v.has_value()) return false;
				m[(size_t)i] = *v;
			}
			out <<
				m[0], m[1], m[2],
				m[3], m[4], m[5],
				m[6], m[7], m[8];
			return true;
		}

		if (value->a.size() == 3) {
			for (int r = 0; r < 3; ++r) {
				const JsonValue& row = value->a[(size_t)r];
				if (!row.IsArray() || row.a.size() != 3) return false;
				for (int c = 0; c < 3; ++c) {
					auto v = JDouble(&row.a[(size_t)c]);
					if (!v.has_value()) return false;
					out(r, c) = *v;
				}
			}
			return true;
		}

		return false;
	}

	std::optional<std::string> XAttr(const XmlNode* node, const std::string& key)
	{
		if (node == nullptr) return std::nullopt;
		return node->Attribute(key);
	}

	std::optional<double> XDouble(const XmlNode* node, const std::string& key)
	{
		auto raw = XAttr(node, key);
		if (!raw.has_value()) return std::nullopt;
		double out = 0.0;
		return ParseDouble(*raw, out) ? std::optional<double>(out) : std::nullopt;
	}

	std::optional<int> XInt(const XmlNode* node, const std::string& key)
	{
		auto raw = XAttr(node, key);
		if (!raw.has_value()) return std::nullopt;
		int out = 0;
		return ParseInt(*raw, out) ? std::optional<int>(out) : std::nullopt;
	}

	std::optional<bool> XBool(const XmlNode* node, const std::string& key)
	{
		auto raw = XAttr(node, key);
		if (!raw.has_value()) return std::nullopt;
		bool out = false;
		return ParseBool(*raw, out) ? std::optional<bool>(out) : std::nullopt;
	}

	bool ParseVec2String(const std::string& value, Eigen::Vector2d& out)
	{
		const std::vector<std::string> tokens = SplitTokens(value);
		if (tokens.size() != 2) return false;
		double x = 0.0, y = 0.0;
		if (!ParseDouble(tokens[0], x) || !ParseDouble(tokens[1], y)) return false;
		out = { x, y };
		return true;
	}

	bool ParseVec3String(const std::string& value, Eigen::Vector3d& out)
	{
		const std::vector<std::string> tokens = SplitTokens(value);
		if (tokens.size() != 3) return false;
		double x = 0.0, y = 0.0, z = 0.0;
		if (!ParseDouble(tokens[0], x) || !ParseDouble(tokens[1], y) || !ParseDouble(tokens[2], z)) return false;
		out = { x, y, z };
		return true;
	}

	bool ParseMat3String(const std::string& value, Eigen::Matrix3d& out)
	{
		const std::vector<std::string> tokens = SplitTokens(value);
		if (tokens.size() != 9) return false;

		std::array<double, 9> m = {};
		for (int i = 0; i < 9; ++i) {
			if (!ParseDouble(tokens[(size_t)i], m[(size_t)i])) return false;
		}

		out <<
			m[0], m[1], m[2],
			m[3], m[4], m[5],
			m[6], m[7], m[8];
		return true;
	}

	void ParseResidualType(const std::string& value, nexdyndiff::ResidualType& out)
	{
		out = ToLower(value) == "force" ? nexdyndiff::ResidualType::Force : nexdyndiff::ResidualType::Acceleration;
	}

	void ParseLinearSolver(const std::string& value, nexdyndiff::LinearSystemSolver& out)
	{
		const std::string lowered = ToLower(value);
		out = (lowered == "directlu" || lowered == "direct_lu") ? nexdyndiff::LinearSystemSolver::DirectLU : nexdyndiff::LinearSystemSolver::CG;
	}

	void ParseSettingsJson(const JsonValue& root, SceneDescription& scene)
	{
		const JsonValue* settings = JGet(&root, "settings");
		if (settings == nullptr || !settings->IsObject()) return;

		const JsonValue* output = JGet(settings, "output");
		if (output != nullptr && output->IsObject()) {
			if (auto v = JString(JGet(output, "simulation_name"))) scene.settings.output.simulation_name = *v;
			if (auto v = JString(JGet(output, "output_directory"))) scene.settings.output.output_directory = *v;
			if (auto v = JString(JGet(output, "codegen_directory"))) scene.settings.output.codegen_directory = *v;
			if (auto v = JInt(JGet(output, "fps"))) scene.settings.output.fps = *v;
			if (auto v = JBool(JGet(output, "enable_output"))) scene.settings.output.enable_output = *v;
		}

		const JsonValue* simulation = JGet(settings, "simulation");
		if (simulation != nullptr && simulation->IsObject()) {
			Eigen::Vector3d gravity = scene.settings.simulation.gravity;
			if (JVec3(JGet(simulation, "gravity"), gravity)) scene.settings.simulation.gravity = gravity;
			if (auto v = JDouble(JGet(simulation, "max_time_step_size"))) scene.settings.simulation.max_time_step_size = *v;
			if (auto v = JBool(JGet(simulation, "use_adaptive_time_step"))) scene.settings.simulation.use_adaptive_time_step = *v;
			if (auto v = JBool(JGet(simulation, "init_frictional_contact"))) scene.settings.simulation.init_frictional_contact = *v;
		}

		const JsonValue* newton = JGet(settings, "newton");
		if (newton != nullptr && newton->IsObject()) {
			if (auto v = JString(JGet(newton, "residual_type"))) ParseResidualType(*v, scene.settings.newton.residual.type);
			if (auto v = JDouble(JGet(newton, "residual_tolerance"))) scene.settings.newton.residual.tolerance = *v;
			if (auto v = JString(JGet(newton, "linear_system_solver"))) ParseLinearSolver(*v, scene.settings.newton.linear_system_solver);
			if (auto v = JInt(JGet(newton, "max_newton_iterations"))) scene.settings.newton.max_newton_iterations = *v;
			if (auto v = JBool(JGet(newton, "project_to_PD"))) scene.settings.newton.project_to_PD = *v;
		}

		const JsonValue* execution = JGet(settings, "execution");
		if (execution != nullptr && execution->IsObject()) {
			if (auto v = JDouble(JGet(execution, "end_simulation_time"))) scene.settings.execution.end_simulation_time = *v;
			if (auto v = JInt(JGet(execution, "n_threads"))) scene.settings.execution.n_threads = *v;
		}
	}

	void ParseGeometryJson(const JsonValue* node, nexdyndiff::scene::GeometryDefinition& geometry)
	{
		if (node == nullptr || !node->IsObject()) return;
		if (auto v = JString(JGet(node, "type"))) geometry.type = ToLower(*v);
		if (auto v = JString(JGet(node, "path"))) geometry.path = *v;
		if (auto v = JString(JGet(node, "generator"))) geometry.generator = ToLower(*v);
		if (auto v = JDouble(JGet(node, "radius"))) geometry.radius = *v;
		if (auto v = JDouble(JGet(node, "height"))) geometry.height = *v;
		if (auto v = JDouble(JGet(node, "outer_radius"))) geometry.outer_radius = *v;
		if (auto v = JDouble(JGet(node, "inner_radius"))) geometry.inner_radius = *v;
		if (auto v = JInt(JGet(node, "slices"))) geometry.slices = *v;
		if (auto v = JInt(JGet(node, "stacks"))) geometry.stacks = *v;
		if (auto v = JInt(JGet(node, "subdivisions"))) geometry.subdivisions = *v;
		if (auto v = JInt(JGet(node, "n_segments"))) geometry.n_segments = *v;
		JVec3(JGet(node, "size"), geometry.size3);
		JVec2(JGet(node, "size"), geometry.size2);
		JInt3(JGet(node, "subdivisions"), geometry.subdivisions3);
		JInt2(JGet(node, "subdivisions"), geometry.subdivisions2);
		JVec3(JGet(node, "begin"), geometry.begin);
		JVec3(JGet(node, "end"), geometry.end);

		const JsonValue* params = JGet(node, "params");
		if (params != nullptr && params->IsObject()) {
			Eigen::Vector2d corner_min2, corner_max2;
			Eigen::Vector3d corner_min3, corner_max3;
			if (JVec2(JGet(params, "corner_min"), corner_min2) && JVec2(JGet(params, "corner_max"), corner_max2)) {
				geometry.center2 = 0.5 * (corner_min2 + corner_max2);
				geometry.size2 = corner_max2 - corner_min2;
			}
			if (JVec3(JGet(params, "corner_min"), corner_min3) && JVec3(JGet(params, "corner_max"), corner_max3)) {
				geometry.center3 = 0.5 * (corner_min3 + corner_max3);
				geometry.size3 = corner_max3 - corner_min3;
			}
			JInt2(JGet(params, "subdivisions"), geometry.subdivisions2);
			JInt3(JGet(params, "subdivisions"), geometry.subdivisions3);
			JVec3(JGet(params, "begin"), geometry.begin);
			JVec3(JGet(params, "end"), geometry.end);
			if (auto v = JInt(JGet(params, "n_segments"))) geometry.n_segments = *v;
		}
	}

	void ParseTransformJson(const JsonValue* node, nexdyndiff::scene::TransformDefinition& transform)
	{
		if (node == nullptr || !node->IsObject()) return;
		JVec3(JGet(node, "translation"), transform.translation);
		JVec3(JGet(node, "rotation"), transform.rotation_deg);
	}

	void ParseScriptedActionJson(const JsonValue* node, nexdyndiff::scene::ScriptedEventActionDefinition& action)
	{
		if (node == nullptr || !node->IsObject()) return;
		if (auto v = JString(JGet(node, "type"))) action.type = ToLower(*v);
		if (auto v = JString(JGet(node, "blend"))) action.blend = ToLower(*v);
		JVec3(JGet(node, "gravity"), action.gravity);
		JVec3(JGet(node, "axis"), action.axis);
		JVec3(JGet(node, "translation"), action.translation);
		JVec3(JGet(node, "translation_velocity"), action.translation_velocity);
		if (auto v = JDouble(JGet(node, "angular_velocity"))) action.angular_velocity = *v;
	}

	void ParseSceneJson(const JsonValue& root, ParseResult& result)
	{
		SceneDescription& scene = result.description;
		if (auto v = JString(JGet(&root, "version"))) scene.version = *v;
		if (const JsonValue* metadata = JGet(&root, "metadata"); metadata != nullptr && metadata->IsObject()) {
			if (auto v = JString(JGet(metadata, "name"))) scene.name = *v;
		}
		ParseSettingsJson(root, scene);

		if (const JsonValue* materials = JGet(&root, "materials"); materials != nullptr && materials->IsObject()) {
			for (const auto& kv : materials->o) {
				if (!kv.second.IsObject()) continue;
				nexdyndiff::scene::MaterialDefinition material;
				material.id = kv.first;
				if (auto v = JString(JGet(&kv.second, "type"))) material.type = ToLower(*v);
				if (auto v = JDouble(JGet(&kv.second, "density"))) material.density = *v;
				if (auto v = JDouble(JGet(&kv.second, "youngs_modulus"))) material.youngs_modulus = *v;
				if (auto v = JDouble(JGet(&kv.second, "poisson_ratio"))) material.poisson_ratio = *v;
				if (auto v = JDouble(JGet(&kv.second, "poissons_ratio"))) material.poisson_ratio = *v;
				if (auto v = JDouble(JGet(&kv.second, "damping"))) material.damping = *v;
				if (auto v = JDouble(JGet(&kv.second, "bending_stiffness"))) material.bending_stiffness = *v;
				if (auto v = JDouble(JGet(&kv.second, "section_radius"))) material.section_radius = *v;
				if (auto v = JBool(JGet(&kv.second, "elasticity_only"))) material.elasticity_only = *v;
				scene.materials.push_back(material);
			}
		}

		const JsonValue* objects = JGet(&root, "objects");
		if (objects != nullptr && objects->IsObject()) {
			if (const JsonValue* deformables = JGet(objects, "deformables"); deformables != nullptr && deformables->IsArray()) {
				for (const JsonValue& node : deformables->a) {
					if (!node.IsObject()) continue;
					nexdyndiff::scene::DeformableDefinition deformable;
					if (auto v = JString(JGet(&node, "id"))) deformable.id = *v;
					if (auto v = JString(JGet(&node, "type"))) deformable.type = ToLower(*v);
					if (auto v = JString(JGet(&node, "label"))) deformable.label = *v;
					if (auto v = JString(JGet(&node, "material"))) deformable.material_id = *v;
					ParseGeometryJson(JGet(&node, "geometry"), deformable.geometry);
					ParseTransformJson(JGet(&node, "transform"), deformable.transform);
					if (const JsonValue* contact = JGet(&node, "contact"); contact != nullptr && contact->IsObject()) {
						if (auto v = JBool(JGet(contact, "enabled"))) deformable.contact_enabled = *v;
						if (auto v = JDouble(JGet(contact, "thickness"))) deformable.contact_thickness = *v;
						if (auto v = JDouble(JGet(contact, "friction"))) deformable.friction = *v;
					}
					if (deformable.label.empty()) deformable.label = deformable.id;
					scene.deformables.push_back(deformable);
				}
			}

			if (const JsonValue* rigid_bodies = JGet(objects, "rigidbodies"); rigid_bodies != nullptr && rigid_bodies->IsArray()) {
				for (const JsonValue& node : rigid_bodies->a) {
					if (!node.IsObject()) continue;
					nexdyndiff::scene::RigidBodyDefinition rigid_body;
					if (auto v = JString(JGet(&node, "id"))) rigid_body.id = *v;
					if (auto v = JString(JGet(&node, "label"))) rigid_body.label = *v;
					if (auto v = JDouble(JGet(&node, "mass"))) rigid_body.mass = *v;
					if (auto v = JBool(JGet(&node, "fixed"))) rigid_body.fixed = *v;
					ParseGeometryJson(JGet(&node, "geometry"), rigid_body.geometry);
					ParseTransformJson(JGet(&node, "transform"), rigid_body.transform);
					Eigen::Vector3d inertia_diagonal = Eigen::Vector3d::Zero();
					if (JVec3(JGet(&node, "inertia_diagonal"), inertia_diagonal)) {
						Eigen::Matrix3d tensor = Eigen::Matrix3d::Zero();
						tensor(0, 0) = inertia_diagonal.x();
						tensor(1, 1) = inertia_diagonal.y();
						tensor(2, 2) = inertia_diagonal.z();
						rigid_body.inertia_tensor = tensor;
					}
					Eigen::Matrix3d inertia_tensor = Eigen::Matrix3d::Zero();
					if (JMat3(JGet(&node, "inertia_tensor"), inertia_tensor)) {
						rigid_body.inertia_tensor = inertia_tensor;
					}
					if (const JsonValue* contact = JGet(&node, "contact"); contact != nullptr && contact->IsObject()) {
						if (auto v = JDouble(JGet(contact, "thickness"))) rigid_body.contact_thickness = *v;
						if (auto v = JDouble(JGet(contact, "friction"))) rigid_body.friction = *v;
					}
					if (rigid_body.label.empty()) rigid_body.label = rigid_body.id;
					scene.rigid_bodies.push_back(rigid_body);
				}
			}
		}

		if (const JsonValue* bcs = JGet(&root, "boundary_conditions"); bcs != nullptr && bcs->IsArray()) {
			for (const JsonValue& node : bcs->a) {
				if (!node.IsObject()) continue;
				nexdyndiff::scene::BoundaryConditionDefinition bc;
				if (auto v = JString(JGet(&node, "id"))) bc.id = *v;
				if (auto v = JString(JGet(&node, "type"))) bc.type = ToLower(*v);
				if (auto v = JString(JGet(&node, "target"))) bc.target = *v;
				if (auto v = JString(JGet(&node, "constraint_type"))) bc.constraint_type = ToLower(*v);
				if (const JsonValue* selector = JGet(&node, "selector"); selector != nullptr && selector->IsObject()) {
					if (auto v = JString(JGet(selector, "type"))) bc.selector.type = ToLower(*v);
					JVec3(JGet(selector, "center"), bc.selector.center);
					JVec3(JGet(selector, "halfsize"), bc.selector.halfsize);
				}
				if (const JsonValue* params = JGet(&node, "params"); params != nullptr && params->IsObject()) {
					if (auto v = JDouble(JGet(params, "stiffness"))) bc.stiffness = *v;
					if (auto v = JDouble(JGet(params, "tolerance"))) bc.tolerance = *v;
					if (auto v = JDouble(JGet(params, "target_distance"))) bc.target_distance = *v;
					if (auto v = JDouble(JGet(params, "min_limit"))) bc.min_limit = *v;
					if (auto v = JDouble(JGet(params, "max_limit"))) bc.max_limit = *v;
					if (auto v = JDouble(JGet(params, "damping"))) bc.damping = *v;
					if (auto v = JDouble(JGet(params, "target_velocity"))) bc.target_velocity = *v;
					if (auto v = JDouble(JGet(params, "max_force"))) bc.max_force = *v;
					if (auto v = JDouble(JGet(params, "admissible_angle"))) bc.admissible_angle_deg = *v;
					if (auto v = JDouble(JGet(params, "admissible_angle_deg"))) bc.admissible_angle_deg = *v;
				}
				if (auto v = JString(JGet(&node, "body_a"))) bc.body_a = *v;
				if (auto v = JString(JGet(&node, "body_b"))) bc.body_b = *v;
				JVec3(JGet(&node, "pivot"), bc.pivot);
				JVec3(JGet(&node, "axis"), bc.axis);
				scene.boundary_conditions.push_back(bc);
			}
		}

		if (const JsonValue* scripted_events = JGet(&root, "scripted_events"); scripted_events != nullptr && scripted_events->IsArray()) {
			for (const JsonValue& node : scripted_events->a) {
				if (!node.IsObject()) continue;
				nexdyndiff::scene::ScriptedEventDefinition event;
				if (auto v = JString(JGet(&node, "type"))) event.type = ToLower(*v);
				if (auto v = JString(JGet(&node, "id"))) event.id = *v;
				if (auto v = JString(JGet(&node, "target"))) event.target = *v;
				if (auto v = JDouble(JGet(&node, "start_time"))) event.start_time = *v;
				if (auto v = JDouble(JGet(&node, "end_time"))) event.end_time = *v;
				ParseScriptedActionJson(JGet(&node, "action"), event.action);
				scene.scripted_events.push_back(event);
			}
		}

		if (const JsonValue* interactions = JGet(&root, "interactions"); interactions != nullptr && interactions->IsObject()) {
			const JsonValue* contact = JGet(interactions, "contact");
			if (contact != nullptr && contact->IsObject()) {
				if (const JsonValue* global = JGet(contact, "global_params"); global != nullptr && global->IsObject()) {
					if (auto v = JDouble(JGet(global, "default_contact_thickness"))) scene.contact_global.default_contact_thickness = *v;
					if (auto v = JDouble(JGet(global, "min_contact_stiffness"))) scene.contact_global.min_contact_stiffness = *v;
					if (auto v = JDouble(JGet(global, "friction_stick_slide_threshold"))) scene.contact_global.friction_stick_slide_threshold = *v;
				}
				if (const JsonValue* pairs = JGet(contact, "pairs"); pairs != nullptr && pairs->IsArray()) {
					for (const JsonValue& pair_node : pairs->a) {
						if (!pair_node.IsObject()) continue;
						nexdyndiff::scene::ContactPairDefinition pair;
						if (auto v = JString(JGet(&pair_node, "object1"))) pair.object1 = *v;
						if (auto v = JString(JGet(&pair_node, "object2"))) pair.object2 = *v;
						if (auto v = JDouble(JGet(&pair_node, "friction"))) pair.friction = *v;
						if (auto v = JBool(JGet(&pair_node, "enabled"))) pair.enabled = *v;
						scene.contact_pairs.push_back(pair);
					}
				}
				if (const JsonValue* disabled = JGet(contact, "disabled_pairs"); disabled != nullptr && disabled->IsArray()) {
					for (const JsonValue& pair_node : disabled->a) {
						nexdyndiff::scene::DisabledContactPairDefinition pair;
						if (pair_node.IsArray() && pair_node.a.size() == 2) {
							if (auto v = JString(&pair_node.a[0])) pair.object1 = *v;
							if (auto v = JString(&pair_node.a[1])) pair.object2 = *v;
						}
						if (!pair.object1.empty() && !pair.object2.empty()) scene.disabled_contact_pairs.push_back(pair);
					}
				}
			}
		}

		if (scene.name.empty()) scene.name = scene.settings.output.simulation_name;
	}

	void ParseGeometryXml(const XmlNode* node, nexdyndiff::scene::GeometryDefinition& geometry)
	{
		if (node == nullptr) return;
		if (auto v = XAttr(node, "type")) geometry.type = ToLower(*v);
		if (auto v = XAttr(node, "path")) geometry.path = *v;
		if (auto v = XAttr(node, "generator")) geometry.generator = ToLower(*v);
		if (auto v = XDouble(node, "radius")) geometry.radius = *v;
		if (auto v = XDouble(node, "height")) geometry.height = *v;
		if (auto v = XDouble(node, "outer_radius")) geometry.outer_radius = *v;
		if (auto v = XDouble(node, "inner_radius")) geometry.inner_radius = *v;
		if (auto v = XInt(node, "slices")) geometry.slices = *v;
		if (auto v = XInt(node, "stacks")) geometry.stacks = *v;
		if (auto v = XInt(node, "subdivisions")) geometry.subdivisions = *v;
		if (auto v = XInt(node, "n_segments")) geometry.n_segments = *v;

		if (const XmlNode* size = node->FindChild("size"); size != nullptr) {
			if (auto x = XDouble(size, "x")) geometry.size3.x() = *x;
			if (auto y = XDouble(size, "y")) geometry.size3.y() = *y;
			if (auto z = XDouble(size, "z")) geometry.size3.z() = *z;
			geometry.size2 = { geometry.size3.x(), geometry.size3.y() };
		}
		if (const XmlNode* subdiv = node->FindChild("subdivisions"); subdiv != nullptr) {
			if (auto nx = XInt(subdiv, "nx")) geometry.subdivisions3[0] = *nx;
			if (auto ny = XInt(subdiv, "ny")) geometry.subdivisions3[1] = *ny;
			if (auto nz = XInt(subdiv, "nz")) geometry.subdivisions3[2] = *nz;
			geometry.subdivisions2 = { geometry.subdivisions3[0], geometry.subdivisions3[1] };
		}
	}

	void ParseTransformXml(const XmlNode* node, nexdyndiff::scene::TransformDefinition& transform)
	{
		if (node == nullptr) return;
		if (auto v = XAttr(node, "translation")) ParseVec3String(*v, transform.translation);
		if (auto v = XAttr(node, "rotation")) ParseVec3String(*v, transform.rotation_deg);
	}

	void ParseScriptedActionXml(const XmlNode* node, nexdyndiff::scene::ScriptedEventActionDefinition& action)
	{
		if (node == nullptr) return;
		if (auto v = XAttr(node, "type")) action.type = ToLower(*v);
		if (auto v = XAttr(node, "blend")) action.blend = ToLower(*v);
		if (auto v = XAttr(node, "gravity")) ParseVec3String(*v, action.gravity);
		if (auto v = XAttr(node, "axis")) ParseVec3String(*v, action.axis);
		if (auto v = XAttr(node, "translation")) ParseVec3String(*v, action.translation);
		if (auto v = XAttr(node, "translation_velocity")) ParseVec3String(*v, action.translation_velocity);
		if (auto v = XDouble(node, "angular_velocity")) action.angular_velocity = *v;
	}

	void ParseSceneXml(const XmlNode& root, ParseResult& result)
	{
		SceneDescription& scene = result.description;
		if (auto v = root.Attribute("version")) scene.version = *v;
		if (const XmlNode* metadata = root.FindChild("metadata"); metadata != nullptr) {
			if (const XmlNode* name = metadata->FindChild("name"); name != nullptr) scene.name = name->text;
		}

		if (const XmlNode* settings = root.FindChild("settings"); settings != nullptr) {
			if (const XmlNode* output = settings->FindChild("output"); output != nullptr) {
				if (auto v = XAttr(output, "simulation_name")) scene.settings.output.simulation_name = *v;
				if (auto v = XAttr(output, "output_directory")) scene.settings.output.output_directory = *v;
				if (auto v = XAttr(output, "codegen_directory")) scene.settings.output.codegen_directory = *v;
				if (auto v = XInt(output, "fps")) scene.settings.output.fps = *v;
				if (auto v = XBool(output, "enable_output")) scene.settings.output.enable_output = *v;
			}
			if (const XmlNode* simulation = settings->FindChild("simulation"); simulation != nullptr) {
				if (auto v = XAttr(simulation, "gravity")) ParseVec3String(*v, scene.settings.simulation.gravity);
				if (auto v = XDouble(simulation, "max_time_step_size")) scene.settings.simulation.max_time_step_size = *v;
				if (auto v = XBool(simulation, "use_adaptive_time_step")) scene.settings.simulation.use_adaptive_time_step = *v;
				if (auto v = XBool(simulation, "init_frictional_contact")) scene.settings.simulation.init_frictional_contact = *v;
			}
			if (const XmlNode* newton = settings->FindChild("newton"); newton != nullptr) {
				if (auto v = XAttr(newton, "residual_type")) ParseResidualType(*v, scene.settings.newton.residual.type);
				if (auto v = XDouble(newton, "residual_tolerance")) scene.settings.newton.residual.tolerance = *v;
				if (auto v = XAttr(newton, "linear_system_solver")) ParseLinearSolver(*v, scene.settings.newton.linear_system_solver);
				if (auto v = XInt(newton, "max_newton_iterations")) scene.settings.newton.max_newton_iterations = *v;
				if (auto v = XBool(newton, "project_to_PD")) scene.settings.newton.project_to_PD = *v;
			}
			if (const XmlNode* execution = settings->FindChild("execution"); execution != nullptr) {
				if (auto v = XDouble(execution, "end_simulation_time")) scene.settings.execution.end_simulation_time = *v;
				if (auto v = XInt(execution, "n_threads")) scene.settings.execution.n_threads = *v;
			}
		}

		if (const XmlNode* materials = root.FindChild("materials"); materials != nullptr) {
			for (const XmlNode* node : materials->FindChildren("material")) {
				nexdyndiff::scene::MaterialDefinition material;
				if (auto v = XAttr(node, "id")) material.id = *v;
				if (auto v = XAttr(node, "type")) material.type = ToLower(*v);
				for (const XmlNode& child : node->children) {
					auto value = XDouble(&child, "value");
					if (!value.has_value()) continue;
					const std::string name = ToLower(child.name);
					if (name == "density") material.density = *value;
					else if (name == "youngs_modulus") material.youngs_modulus = *value;
					else if (name == "poisson_ratio" || name == "poissons_ratio") material.poisson_ratio = *value;
					else if (name == "damping") material.damping = *value;
					else if (name == "bending_stiffness") material.bending_stiffness = *value;
					else if (name == "section_radius") material.section_radius = *value;
				}
				scene.materials.push_back(material);
			}
		}

		const XmlNode* objects = root.FindChild("objects");
		if (objects != nullptr) {
			if (const XmlNode* deformables = objects->FindChild("deformables"); deformables != nullptr) {
				for (const XmlNode* node : deformables->FindChildren("deformable")) {
					nexdyndiff::scene::DeformableDefinition deformable;
					if (auto v = XAttr(node, "id")) deformable.id = *v;
					if (auto v = XAttr(node, "type")) deformable.type = ToLower(*v);
					if (auto v = XAttr(node, "label")) deformable.label = *v;
					if (auto v = XAttr(node, "material")) deformable.material_id = *v;
					ParseGeometryXml(node->FindChild("geometry"), deformable.geometry);
					ParseTransformXml(node->FindChild("transform"), deformable.transform);
					if (const XmlNode* contact = node->FindChild("contact"); contact != nullptr) {
						if (auto v = XBool(contact, "enabled")) deformable.contact_enabled = *v;
						if (auto v = XDouble(contact, "thickness")) deformable.contact_thickness = *v;
						if (auto v = XDouble(contact, "friction")) deformable.friction = *v;
					}
					if (deformable.label.empty()) deformable.label = deformable.id;
					scene.deformables.push_back(deformable);
				}
			}
			if (const XmlNode* rigid_bodies = objects->FindChild("rigidbodies"); rigid_bodies != nullptr) {
				for (const XmlNode* node : rigid_bodies->FindChildren("rigidbody")) {
					nexdyndiff::scene::RigidBodyDefinition rigid_body;
					if (auto v = XAttr(node, "id")) rigid_body.id = *v;
					if (auto v = XAttr(node, "label")) rigid_body.label = *v;
					if (auto v = XDouble(node, "mass")) rigid_body.mass = *v;
					if (auto v = XBool(node, "fixed")) rigid_body.fixed = *v;
					ParseGeometryXml(node->FindChild("geometry"), rigid_body.geometry);
					ParseTransformXml(node->FindChild("transform"), rigid_body.transform);
					if (auto v = XAttr(node, "inertia_diagonal")) {
						Eigen::Vector3d diagonal = Eigen::Vector3d::Zero();
						if (ParseVec3String(*v, diagonal)) {
							Eigen::Matrix3d tensor = Eigen::Matrix3d::Zero();
							tensor(0, 0) = diagonal.x();
							tensor(1, 1) = diagonal.y();
							tensor(2, 2) = diagonal.z();
							rigid_body.inertia_tensor = tensor;
						}
					}
					if (auto v = XAttr(node, "inertia_tensor")) {
						Eigen::Matrix3d tensor = Eigen::Matrix3d::Zero();
						if (ParseMat3String(*v, tensor)) {
							rigid_body.inertia_tensor = tensor;
						}
					}
					if (const XmlNode* inertia = node->FindChild("inertia"); inertia != nullptr) {
						if (auto v = XAttr(inertia, "diagonal")) {
							Eigen::Vector3d diagonal = Eigen::Vector3d::Zero();
							if (ParseVec3String(*v, diagonal)) {
								Eigen::Matrix3d tensor = Eigen::Matrix3d::Zero();
								tensor(0, 0) = diagonal.x();
								tensor(1, 1) = diagonal.y();
								tensor(2, 2) = diagonal.z();
								rigid_body.inertia_tensor = tensor;
							}
						}
						if (auto v = XAttr(inertia, "tensor")) {
							Eigen::Matrix3d tensor = Eigen::Matrix3d::Zero();
							if (ParseMat3String(*v, tensor)) {
								rigid_body.inertia_tensor = tensor;
							}
						}
					}
					if (const XmlNode* contact = node->FindChild("contact"); contact != nullptr) {
						if (auto v = XDouble(contact, "thickness")) rigid_body.contact_thickness = *v;
						if (auto v = XDouble(contact, "friction")) rigid_body.friction = *v;
					}
					if (rigid_body.label.empty()) rigid_body.label = rigid_body.id;
					scene.rigid_bodies.push_back(rigid_body);
				}
			}
		}

		if (const XmlNode* bcs = root.FindChild("boundary_conditions"); bcs != nullptr) {
			for (const XmlNode& node : bcs->children) {
				nexdyndiff::scene::BoundaryConditionDefinition bc;
				if (auto v = XAttr(&node, "id")) bc.id = *v;
				bc.type = ToLower(node.name);
				if (auto v = XAttr(&node, "target")) bc.target = *v;
				if (bc.type == "rigidbody_constraint") {
					if (auto v = XAttr(&node, "type")) bc.constraint_type = ToLower(*v);
					if (auto v = XAttr(&node, "body_a")) bc.body_a = *v;
					if (auto v = XAttr(&node, "body_b")) bc.body_b = *v;
					if (auto v = XAttr(&node, "pivot")) ParseVec3String(*v, bc.pivot);
					if (auto v = XAttr(&node, "axis")) ParseVec3String(*v, bc.axis);
					if (auto v = XDouble(&node, "target_distance")) bc.target_distance = *v;
					if (auto v = XDouble(&node, "min_limit")) bc.min_limit = *v;
					if (auto v = XDouble(&node, "max_limit")) bc.max_limit = *v;
					if (auto v = XDouble(&node, "damping")) bc.damping = *v;
					if (auto v = XDouble(&node, "target_velocity")) bc.target_velocity = *v;
					if (auto v = XDouble(&node, "max_force")) bc.max_force = *v;
					if (auto v = XDouble(&node, "admissible_angle")) bc.admissible_angle_deg = *v;
					if (auto v = XDouble(&node, "admissible_angle_deg")) bc.admissible_angle_deg = *v;
				}
				if (bc.type == "prescribed_position") {
					if (const XmlNode* selector = node.FindChild("selector"); selector != nullptr) {
						if (auto v = XAttr(selector, "type")) bc.selector.type = ToLower(*v);
						if (auto v = XAttr(selector, "center")) ParseVec3String(*v, bc.selector.center);
						if (auto v = XAttr(selector, "halfsize")) ParseVec3String(*v, bc.selector.halfsize);
					}
					if (const XmlNode* params = node.FindChild("params"); params != nullptr) {
						if (auto v = XDouble(params, "stiffness")) bc.stiffness = *v;
						if (auto v = XDouble(params, "tolerance")) bc.tolerance = *v;
					}
				}
				scene.boundary_conditions.push_back(bc);
			}
		}

		if (const XmlNode* scripted_events = root.FindChild("scripted_events"); scripted_events != nullptr) {
			for (const XmlNode& node : scripted_events->children) {
				nexdyndiff::scene::ScriptedEventDefinition event;
				event.type = ToLower(node.name);
				if (auto v = XAttr(&node, "id")) event.id = *v;
				if (auto v = XAttr(&node, "target")) event.target = *v;
				if (auto v = XDouble(&node, "start_time")) event.start_time = *v;
				if (auto v = XDouble(&node, "end_time")) event.end_time = *v;
				ParseScriptedActionXml(node.FindChild("action"), event.action);
				scene.scripted_events.push_back(event);
			}
		}

		if (const XmlNode* interactions = root.FindChild("interactions"); interactions != nullptr) {
			const XmlNode* contact = interactions->FindChild("contact");
			if (contact != nullptr) {
				if (const XmlNode* global = contact->FindChild("global_params"); global != nullptr) {
					if (auto v = XDouble(global, "default_contact_thickness")) scene.contact_global.default_contact_thickness = *v;
					if (auto v = XDouble(global, "min_contact_stiffness")) scene.contact_global.min_contact_stiffness = *v;
					if (auto v = XDouble(global, "friction_stick_slide_threshold")) scene.contact_global.friction_stick_slide_threshold = *v;
				}
				for (const XmlNode* pair_node : contact->FindChildren("friction_pair")) {
					nexdyndiff::scene::ContactPairDefinition pair;
					if (auto v = XAttr(pair_node, "object1")) pair.object1 = *v;
					if (auto v = XAttr(pair_node, "object2")) pair.object2 = *v;
					if (auto v = XDouble(pair_node, "friction")) pair.friction = *v;
					if (auto v = XBool(pair_node, "enabled")) pair.enabled = *v;
					scene.contact_pairs.push_back(pair);
				}
				for (const XmlNode* pair_node : contact->FindChildren("disabled_pair")) {
					nexdyndiff::scene::DisabledContactPairDefinition pair;
					if (auto v = XAttr(pair_node, "object1")) pair.object1 = *v;
					if (auto v = XAttr(pair_node, "object2")) pair.object2 = *v;
					if (!pair.object1.empty() && !pair.object2.empty()) scene.disabled_contact_pairs.push_back(pair);
				}
			}
		}

		if (scene.name.empty()) scene.name = scene.settings.output.simulation_name;
	}

	void ValidateScene(ParseResult& result)
	{
		const SceneDescription& scene = result.description;
		std::unordered_set<std::string> material_ids;
		for (const auto& material : scene.materials) if (!material.id.empty()) material_ids.insert(material.id);

		std::unordered_set<std::string> object_ids;
		for (const auto& deformable : scene.deformables) {
			if (deformable.id.empty()) {
				AddError(result, "objects.deformables", "Deformable id cannot be empty");
			}
			else if (!object_ids.insert(deformable.id).second) {
				AddError(result, "objects.deformables." + deformable.id, "Duplicate object id");
			}
			if (!deformable.material_id.empty() && material_ids.find(deformable.material_id) == material_ids.end()) {
				AddError(result, "objects.deformables." + deformable.id + ".material", "Unknown material reference: " + deformable.material_id);
			}
		}
		for (const auto& rigid_body : scene.rigid_bodies) {
			if (rigid_body.id.empty()) {
				AddError(result, "objects.rigidbodies", "RigidBody id cannot be empty");
			}
			else if (!object_ids.insert(rigid_body.id).second) {
				AddError(result, "objects.rigidbodies." + rigid_body.id, "Duplicate object id");
			}
		}
		for (const auto& bc : scene.boundary_conditions) {
			if (!bc.target.empty() && object_ids.find(bc.target) == object_ids.end()) {
				AddError(result, "boundary_conditions", "Unknown target: " + bc.target);
			}
		}
	}

	bool ReadTextFile(const std::filesystem::path& file_path, std::string& out_text)
	{
		std::ifstream file(file_path, std::ios::in | std::ios::binary);
		if (!file.is_open()) return false;
		std::ostringstream buffer;
		buffer << file.rdbuf();
		out_text = buffer.str();
		return true;
	}
}

std::optional<nexdyndiff::scene::SceneFileFormat> nexdyndiff::scene::SceneParser::DetectFormat(const std::filesystem::path& file_path)
{
	const std::string ext = detail::ToLower(file_path.extension().string());
	if (ext == ".json") return SceneFileFormat::Json;
	if (ext == ".xml" || ext == ".scene") return SceneFileFormat::Xml;

	std::string text;
	if (!ReadTextFile(file_path, text)) return std::nullopt;
	for (char c : text) {
		if (std::isspace((unsigned char)c)) continue;
		if (c == '{' || c == '[') return SceneFileFormat::Json;
		if (c == '<') return SceneFileFormat::Xml;
		break;
	}
	return std::nullopt;
}

nexdyndiff::scene::ParseResult nexdyndiff::scene::SceneParser::ParseFile(const std::filesystem::path& file_path)
{
	ParseResult result;
	std::string text;
	if (!ReadTextFile(file_path, text)) {
		AddError(result, file_path.string(), "Unable to read scene file");
		return result;
	}
	const std::optional<SceneFileFormat> format = DetectFormat(file_path);
	if (!format.has_value()) {
		AddError(result, file_path.string(), "Unable to detect scene format");
		return result;
	}
	return ParseText(text, *format, file_path.parent_path());
}

nexdyndiff::scene::ParseResult nexdyndiff::scene::SceneParser::ParseText(
	const std::string& text,
	SceneFileFormat format,
	const std::filesystem::path&)
{
	ParseResult result;
	if (format == SceneFileFormat::Json) {
		detail::JsonParser parser(text);
		detail::JsonValue root;
		std::string err;
		if (!parser.Parse(root, err)) {
			AddError(result, "json", err);
			return result;
		}
		if (!root.IsObject()) {
			AddError(result, "json", "Root JSON node must be an object");
			return result;
		}
		ParseSceneJson(root, result);
	}
	else {
		detail::XmlParser parser(text);
		detail::XmlNode root;
		std::string err;
		if (!parser.Parse(root, err)) {
			AddError(result, "xml", err);
			return result;
		}
		if (detail::ToLower(root.name) != "scene") {
			AddError(result, "xml", "Root XML node must be <scene>");
			return result;
		}
		ParseSceneXml(root, result);
	}

	ValidateScene(result);
	return result;
}
