#include "RigidBodiesMeshOutput.h"

#include "../../utils/include.h"


nexdyndiff::RigidBodiesMeshOutput::RigidBodiesMeshOutput(core::NexDynDiff& nexdyndiff, spRigidBodyDynamics rb)
	: rb(rb)
{
	nexdyndiff.callbacks.add_write_frame([&]() { this->_write_frame(nexdyndiff); });
}
void nexdyndiff::RigidBodiesMeshOutput::add_point_mesh(const std::string& label, const RigidBodyHandler& rb, const std::vector<Eigen::Vector3d>& vertices)
{
	rb.exit_if_not_valid("RigidBodiesMeshOutput::add_point_set");
	std::vector<std::array<int, 1>> conn(vertices.size());
	for (int i = 0; i < (int)vertices.size(); i++) {
		conn[i] = { i };
	}

	this->points.push_back({ rb, vertices });
	this->point_groups.add_to_group(label, (int)this->points.size() - 1);
}
void nexdyndiff::RigidBodiesMeshOutput::add_segment_mesh(const std::string& label, const RigidBodyHandler& rb, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 2>>& conn)
{
	rb.exit_if_not_valid("RigidBodiesMeshOutput::add_segment_mesh");
	this->segments.push_back({ rb, vertices, conn });
	this->segment_groups.add_to_group(label, (int)this->segments.size() - 1);
}
void nexdyndiff::RigidBodiesMeshOutput::add_triangle_mesh(const std::string& label, const RigidBodyHandler& rb, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& conn)
{
	rb.exit_if_not_valid("RigidBodiesMeshOutput::add_triangle_mesh");
	this->triangles.push_back({ rb, vertices, conn });
	this->triangle_groups.add_to_group(label, (int)this->triangles.size() - 1);
}
void nexdyndiff::RigidBodiesMeshOutput::add_tet_mesh(const std::string& label, const RigidBodyHandler& rb, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 4>>& conn)
{
	rb.exit_if_not_valid("RigidBodiesMeshOutput::add_tet_mesh");
	this->tets.push_back({ rb, vertices, conn });
	this->tet_groups.add_to_group(label, (int)this->tets.size() - 1);
}

void nexdyndiff::RigidBodiesMeshOutput::_write_frame(nexdyndiff::core::NexDynDiff& nexdyndiff)
{
	this->_write(nexdyndiff, this->point_groups, this->points, conn_points);
	this->_write(nexdyndiff, this->segment_groups, this->segments, conn_segments);
	this->_write(nexdyndiff, this->triangle_groups, this->triangles, conn_triangles);
	this->_write(nexdyndiff, this->tet_groups, this->tets, conn_tets);
}

template<std::size_t N>
inline void nexdyndiff::RigidBodiesMeshOutput::_write(core::NexDynDiff& nexdyndiff, const MeshOutputGroups& groups, const std::vector<Mesh<N>>& meshes, std::vector<std::array<int, N>>& conn_buffer)
{
	for (const auto& it : groups.groups) {
		const std::string& label = it.first;
		const std::unordered_set<int>& group = it.second;

		this->vertices.clear();
		conn_buffer.clear();
		for (const int local_idx : group) {
			const auto& mesh = meshes[local_idx];
			const auto rb = mesh.rb;

			const int out_vertex_offset = (int)this->vertices.size();
			for (const Eigen::Vector3d& vertex : mesh.vertices) {
				this->vertices.push_back(rb.transform_local_to_global_point(vertex));
			}

			for (const auto& elem : mesh.conn) {
				conn_buffer.push_back(elem);
				for (int i = 0; i < N; i++) {
					conn_buffer.back()[i] += out_vertex_offset;
				}
			}
		}
		nexdyndiff::write_VTK(nexdyndiff.get_frame_path(label) + ".vtk", this->vertices, conn_buffer);
	}
}
// Forward declaration
template void nexdyndiff::RigidBodiesMeshOutput::_write<1>(nexdyndiff::core::NexDynDiff& nexdyndiff, const nexdyndiff::MeshOutputGroups& groups, const std::vector<nexdyndiff::RigidBodiesMeshOutput::Mesh<1>>& meshes, std::vector<std::array<int, 1>>& conn_buffer);
template void nexdyndiff::RigidBodiesMeshOutput::_write<2>(nexdyndiff::core::NexDynDiff& nexdyndiff, const nexdyndiff::MeshOutputGroups& groups, const std::vector<nexdyndiff::RigidBodiesMeshOutput::Mesh<2>>& meshes, std::vector<std::array<int, 2>>& conn_buffer);
template void nexdyndiff::RigidBodiesMeshOutput::_write<3>(nexdyndiff::core::NexDynDiff& nexdyndiff, const nexdyndiff::MeshOutputGroups& groups, const std::vector<nexdyndiff::RigidBodiesMeshOutput::Mesh<3>>& meshes, std::vector<std::array<int, 3>>& conn_buffer);
template void nexdyndiff::RigidBodiesMeshOutput::_write<4>(nexdyndiff::core::NexDynDiff& nexdyndiff, const nexdyndiff::MeshOutputGroups& groups, const std::vector<nexdyndiff::RigidBodiesMeshOutput::Mesh<4>>& meshes, std::vector<std::array<int, 4>>& conn_buffer);
