#include "MeshingVoxelGrid.h"
#include <queue>
#include <vector>
#include <iostream>
#include <chrono>
#include <unordered_set>

#define DEBUG_ENABLED_MVG 0
//#define DEBUG_ENABLED_MVG 1

#define MESH_COLORS_ENABLED_MVG 0
//#define MESH_COLORS_ENABLED_MVG 1

MeshingVoxelGrid::MeshingVoxelGrid(MeshingVoxelParams& params)
{
	SetData(params);
}

MeshingVoxelGrid::MeshingVoxelGrid()
{
}

MeshingVoxelGrid::~MeshingVoxelGrid()
{
	delete[] grid;
	grid = nullptr;
}

void MeshingVoxelGrid::SetData(MeshingVoxelParams& params)
{
	if (grid != nullptr)
	{
		delete[] grid;
		grid = nullptr;
	}

	this->params = params;

	grid = new SingleVoxel[params.points_x * params.points_y * params.points_z];

	first_corner.x() = params.center.x() - params.voxel_size * 0.5 * (double)(params.points_x - 1);
	first_corner.y() = params.center.y() - params.voxel_size * 0.5 * (double)(params.points_y - 1);
	first_corner.z() = params.center.z() - params.voxel_size * 0.5 * (double)(params.points_z - 1);

#if DEBUG_ENABLED_MVG == 1
	std::cout << "First corner: " << first_corner.x() << ", " << first_corner.y() << ", " << first_corner.z() << std::endl;

	std::cout << "Center: " << params.center.x() << ", " << params.center.y() << ", " << params.center.z() << std::endl;
#endif
}

void MeshingVoxelGrid::AddImage(open3d::geometry::Image& color, open3d::geometry::Image& depth, Eigen::Matrix4d extrinsics, Eigen::Matrix3d intrinsics)
{
	auto depth_float = depth.ConvertDepthToFloatImage();

	int culled = 0;
	int solid = 0;
	int air = 0;
	int is_zero = 0;

	int grid_loc = 0;

	Eigen::Matrix3d rotation = extrinsics.block<3, 3>(0, 0);
	Eigen::Vector3d position = extrinsics.block<3, 1>(0, 3);

	Eigen::Matrix3d rotation_inv = rotation.inverse();

	Eigen::Matrix3d intrinsic_inv = intrinsics.inverse();

	Eigen::Vector3d voxel_position;

	for (int x = 0; x < params.points_x; ++x)
	{
		for (int y = 0; y < params.points_y; ++y)
		{
			for (int z = 0; z < params.points_z; ++z, ++grid_loc)
			{
				voxel_position = first_corner + params.voxel_size * Eigen::Vector3d(x, y, z);

				auto voxel = &grid[grid_loc];

				Eigen::Vector3d uvz = intrinsics *
					(rotation * voxel_position + position);

				double pix_u = uvz.x() / (uvz.z());
				double pix_v = uvz.y() / (uvz.z());

				double pixel_depth = 0;
				bool in_bounds;
				std::tie(in_bounds, pixel_depth) = depth_float->FloatValueAt(pix_u, pix_v);

				if (!in_bounds)
				{
					++culled;
					continue;
				}

				int int_x = std::clamp((int)(pix_u * color.width_), 0, color.width_ - 1);
				int int_y = std::clamp((int)(pix_v * color.height_), 0, color.height_ - 1);
				
				//auto color_r = *color.PointerAt<uint8_t>(int_x, int_y, 0);
				//auto color_g = *color.PointerAt<uint8_t>(int_x, int_y, 1);
				//auto color_b = *color.PointerAt<uint8_t>(int_x, int_y, 2);

				auto color_r = *color.PointerAt<uint8_t>(pix_u, pix_v, 0);
				auto color_g = *color.PointerAt<uint8_t>(pix_u, pix_v, 1);
				auto color_b = *color.PointerAt<uint8_t>(pix_u, pix_v, 2);

				Eigen::Vector3d pixel_position = rotation_inv * (intrinsic_inv * Eigen::Vector3d(uvz.x(), uvz.y(), pixel_depth) - position);

				Eigen::Vector3d dist = (pixel_position - voxel_position);
				double mag = sqrt(dist.dot(dist)) / params.voxel_size;
				mag = std::min(mag, 1.0);

				//For future reference, this is a signed distance field

				//AIR
				if (pixel_depth > uvz.z() || pixel_depth == 0)
				{
					//if (pixel_depth == 0)
					//{
					//	++is_zero;
					//}

					if (voxel->voxel_type == MeshingVoxelType::SOLID)
					{
						voxel->voxel_type = MeshingVoxelType::AIR;
						voxel->value = mag; // std::min(mag, 1.0);
					}
					else if (voxel->voxel_type == MeshingVoxelType::AIR)
					{
						voxel->value = std::max(voxel->value, mag);
					}
					else
					{
						voxel->voxel_type = MeshingVoxelType::AIR;
						voxel->value = std::max(voxel->value, mag);
					}

					++air;
				}
				//SOLID
				else
				{
					if (voxel->voxel_type == MeshingVoxelType::SOLID)
					{
						if (voxel->value > mag)
						{
							voxel->value = std::min(voxel->value, mag);
							//voxel->color = Eigen::Vector3d((double)color_r / 255.0, (double)color_g / 255.0, (double)color_b / 255.0);
						}

						++solid;
					}
					else if (voxel->voxel_type == MeshingVoxelType::AIR)
					{

					}
					else
					{
						voxel->voxel_type = MeshingVoxelType::SOLID;
						//voxel->color = Eigen::Vector3d((double)color_r / 255.0, (double)color_g / 255.0, (double)color_b / 255.0);

						//std::cout << voxel->color.x() << ", " << voxel->color.y() << ", " << voxel->color.z() << ", " << int_x << ", " << int_y << std::endl;

						voxel->value = mag; //std::min(mag, 1.0);
						
						++solid;
					}
				}
			}
		}
	}

#if DEBUG_ENABLED_MVG == 1
	std::cout << "culled voxels: " << culled << "/" << (params.points_x * params.points_y * params.points_z) << std::endl;
	std::cout << "solid voxels: " << solid << "/" << (params.points_x * params.points_y * params.points_z) << std::endl;
	std::cout << "air voxels: " << air << "/" << (params.points_x * params.points_y * params.points_z) << std::endl;
	std::cout << "zero depth voxels: " << is_zero << "/" << (params.points_x * params.points_y * params.points_z) << std::endl;
#endif
}

void MeshingVoxelGrid::KillEmptySpace()
{
	int empty_voxels = 0;
	int filled_solid = 0;
	int filled_air = 0;

	int grid_loc = 0;

	int vote_solid = 0;

	int step_x = params.points_z * params.points_y;
	int step_y = params.points_z;
	int step_z = 1;

	bool upper_bound_x;
	bool lower_bound_x;
	bool upper_bound_y;
	bool lower_bound_y;
	bool upper_bound_z;
	bool lower_bound_z;

	for (int x = 0; x < params.points_x; ++x)
	{
		upper_bound_x = (x == params.points_x - 1);
		lower_bound_x = (x == 0);

		for (int y = 0; y < params.points_y; ++y)
		{
			upper_bound_y = (y == params.points_y - 1);
			lower_bound_y = (y == 0);

			for (int z = 0; z < params.points_z; ++z, ++grid_loc)
			{
				if (grid[grid_loc].voxel_type != MeshingVoxelType::NONE)
				{
					continue;
				}

				upper_bound_z = (z == params.points_z - 1);
				lower_bound_z = (z == 0);

				++empty_voxels;

				vote_solid += (grid[grid_loc + step_x * (!upper_bound_x)].value == MeshingVoxelType::SOLID);
				vote_solid += (grid[grid_loc - step_x * (!lower_bound_x)].value == MeshingVoxelType::SOLID);
				vote_solid += (grid[grid_loc + step_y * (!upper_bound_y)].value == MeshingVoxelType::SOLID);
				vote_solid += (grid[grid_loc - step_y * (!lower_bound_y)].value == MeshingVoxelType::SOLID);
				vote_solid += (grid[grid_loc + step_z * (!upper_bound_z)].value == MeshingVoxelType::SOLID);
				vote_solid += (grid[grid_loc - step_z * (!lower_bound_z)].value == MeshingVoxelType::SOLID);

				if (vote_solid >= 3)
				{
					grid[grid_loc].voxel_type = MeshingVoxelType::SOLID;
					grid[grid_loc].value = (double)vote_solid / 6.0;

					++filled_solid;
				}
				else
				{
					grid[grid_loc].voxel_type = MeshingVoxelType::AIR;
					
					++filled_air;
				}
			}
		}
	}

#if DEBUG_ENABLED_MVG == 1
	std::cout << empty_voxels << " empty voxels found: filled " << filled_solid << " solid, filled " << filled_air << " air" << std::endl;
#endif
}

std::shared_ptr<open3d::geometry::TriangleMesh> MeshingVoxelGrid::ExtractMesh()
{
	//KillEmptySpace();

	auto to_return = std::make_shared<open3d::geometry::TriangleMesh>();

	int grid_loc = 0;
	int solid_voxels = 0;

	for (int x = 0; x < params.points_x; ++x)
	{
		for (int y = 0; y < params.points_y; ++y)
		{
			for (int z = 0; z < params.points_z; ++z, ++grid_loc)
			{
				solid_voxels += (grid[grid_loc].voxel_type == MeshingVoxelType::SOLID);
			}
		}
	}

#if DEBUG_ENABLED_MVG == 1
	std::cout << "Solid voxels total: " << solid_voxels << "/" << (params.points_x * params.points_y * params.points_z) << std::endl;
#endif
	int index_count = 0;

	int corners[8];
	std::tuple<int, int> edges[12];

	//std::vector<int> marching_cubes_indicies;

	std::map<std::tuple<int, int>, MeshingVoxelEdge> created_edges;
	std::vector<std::tuple<int, int>> keys;

	int step_x = params.points_z * params.points_y;
	int step_y = params.points_z;
	int step_z = 1;

	int no_triangles = 0;

	Eigen::Vector3d positions[8];
	Eigen::Vector3d right = Eigen::Vector3d(params.voxel_size, 0, 0);
	Eigen::Vector3d up = Eigen::Vector3d(0, params.voxel_size, 0);
	Eigen::Vector3d forward = Eigen::Vector3d(0, 0, params.voxel_size);

	for (int x = 0; x < params.points_x - 1; ++x)
	{
		for (int y = 0; y < params.points_y - 1; ++y)
		{
			for (int z = 0; z < params.points_z - 1; ++z)
			{
				//The corners of the grid
				corners[0] = z * step_z + y * step_y + x * step_x;
				corners[4] = corners[0] + step_z;
				corners[2] = corners[0] + step_y;
				corners[6] = corners[2] + step_z;
				corners[1] = corners[0] + step_x;
				corners[5] = corners[1] + step_z;
				corners[3] = corners[1] + step_y;
				corners[7] = corners[3] + step_z;

				positions[0] = first_corner + params.voxel_size * Eigen::Vector3d(x, y, z);
				positions[4] = positions[0] + forward;
				positions[2] = positions[0] + up;
				positions[6] = positions[2] + forward;
				positions[1] = positions[0] + right;
				positions[5] = positions[1] + forward;
				positions[3] = positions[1] + up;
				positions[7] = positions[3] + forward;

				//Testing which edges will be produced
				int index = 0;

				index |= 1 *	(grid[corners[0]].voxel_type == MeshingVoxelType::SOLID);
				index |= 2 *	(grid[corners[1]].voxel_type == MeshingVoxelType::SOLID);
				index |= 8 *	(grid[corners[2]].voxel_type == MeshingVoxelType::SOLID);
				index |= 4 *	(grid[corners[3]].voxel_type == MeshingVoxelType::SOLID);
				index |= 16 *	(grid[corners[4]].voxel_type == MeshingVoxelType::SOLID);
				index |= 32 *	(grid[corners[5]].voxel_type == MeshingVoxelType::SOLID);
				index |= 128 *	(grid[corners[6]].voxel_type == MeshingVoxelType::SOLID);
				index |= 64 *	(grid[corners[7]].voxel_type == MeshingVoxelType::SOLID);

				index = 255 - index;

				if (edge_table[index] == 0)
				{
					++no_triangles;
					continue;
				}

				//marching_cubes_indicies.push_back(index);

				//Interpolating edges
				if ((edge_table[index] & 1) > 0)
					edges[0] = LerpCorner(grid, created_edges, keys, corners[0], corners[1], positions[0], positions [1]);
				if ((edge_table[index] & 2) > 0)
					edges[1] = LerpCorner(grid, created_edges, keys, corners[1], corners[3], positions[1], positions[3]);
				if ((edge_table[index] & 4) > 0)
					edges[2] = LerpCorner(grid, created_edges, keys, corners[3], corners[2], positions[3], positions[2]);
				if ((edge_table[index] & 8) > 0)
					edges[3] = LerpCorner(grid, created_edges, keys, corners[2], corners[0], positions[2], positions[0]);
				if ((edge_table[index] & 16) > 0)
					edges[4] = LerpCorner(grid, created_edges, keys, corners[4], corners[5], positions[4], positions[5]);
				if ((edge_table[index] & 32) > 0)
					edges[5] = LerpCorner(grid, created_edges, keys, corners[5], corners[7], positions[5], positions[7]);
				if ((edge_table[index] & 64) > 0)
					edges[6] = LerpCorner(grid, created_edges, keys, corners[7], corners[6], positions[7], positions[6]);
				if ((edge_table[index] & 128) > 0)
					edges[7] = LerpCorner(grid, created_edges, keys, corners[6], corners[4], positions[6], positions[4]);
				if ((edge_table[index] & 256) > 0)
					edges[8] = LerpCorner(grid, created_edges, keys, corners[0], corners[4], positions[0], positions[4]);
				if ((edge_table[index] & 512) > 0)
					edges[9] = LerpCorner(grid, created_edges, keys, corners[1], corners[5], positions[1], positions[5]);
				if ((edge_table[index] & 1024) > 0)
					edges[10] = LerpCorner(grid, created_edges, keys, corners[3], corners[7], positions[3], positions[7]);
				if ((edge_table[index] & 2048) > 0)
					edges[11] = LerpCorner(grid, created_edges, keys, corners[2], corners[6], positions[2], positions[6]);

				auto tri_table_seg = tri_table[index];

				//Adding triangles to the mesh
				for (int i = 0; tri_table_seg[i] != -1; i += 3)
				{
					auto edge0 = created_edges[edges[tri_table_seg[i]]];
					auto edge1 = created_edges[edges[tri_table_seg[i + 1]]];
					auto edge2 = created_edges[edges[tri_table_seg[i + 2]]];

					to_return->triangles_.push_back(
						Eigen::Vector3i(edge0.index, edge1.index, edge2.index)
					);
				}
			}
		}
	}

	to_return->vertices_.resize(keys.size());
#if MESH_COLORS_ENABLED_MVG == 1
	to_return->vertex_colors_.resize(keys.size());
#endif
	Eigen::Vector3d grid_dims = Eigen::Vector3d(params.points_x, params.points_y, params.points_z) * params.voxel_size;

	for (int i = 0; i < keys.size(); ++i)
	{
		auto edge = created_edges[keys[i]];
		to_return->vertices_[edge.index] = edge.position;

#if MESH_COLORS_ENABLED_MVG == 1
		Eigen::Vector3d col = (edge.position - first_corner);

		col.x() /= grid_dims.x();
		col.y() /= grid_dims.y();
		col.z() /= grid_dims.z();

		to_return->vertex_colors_[edge.index] = col;
#endif
	}

#if DEBUG_ENABLED_MVG == 1
	std::cout << "Mesh vertices: " << to_return->vertices_.size() << std::endl;
	std::cout << "Voxels without triangles: " << no_triangles << "/" << ((params.points_x - 1) * (params.points_y - 1) * (params.points_z - 1)) << std::endl;
#endif
	return to_return;
}

std::tuple<int, int> MeshingVoxelGrid::LerpCorner(SingleVoxel* voxel_array, std::map<std::tuple<int, int>, MeshingVoxelEdge>& edge_map, 
	std::vector<std::tuple<int, int>>& keys, int elem1, int elem2, Eigen::Vector3d &pos1, Eigen::Vector3d &pos2)
{
	auto elem_tuple = std::make_tuple(elem1, elem2);

	std::map<std::tuple<int, int>, MeshingVoxelEdge>::iterator it = edge_map.find(elem_tuple);

	if (it != edge_map.end())
	{
		return elem_tuple;
	}

	double t = voxel_array[elem1].value / (voxel_array[elem1].value + voxel_array[elem2].value);

	Eigen::Vector3d final_pos = pos1 * (1.0 - t) + pos2 * t;

	auto new_edge = MeshingVoxelEdge(final_pos, keys.size());

	keys.push_back(elem_tuple);

	edge_map[elem_tuple] = new_edge;

	return elem_tuple;
}

void MeshingVoxelGrid::CullArtifacts(int artifact_size)
{
	//if (artifact_size <= 0)
	//{
	//	return;
	//}
	//
	//int grid_loc = 0;
	//int current = 0;
	//int limit = 0;
	//
	//int step_x = params.points_z * params.points_y;
	//int step_y = params.points_z;
	//int step_z = 1;
	//
	//int x_lower = 0;
	//int x_upper = 0;
	//
	//int y_lower = 0;
	//int y_upper = 0;
	//
	//int z_lower = 0;
	//int z_upper = 0;
	//
	//bool cull_artifacts_harsh = true;
	//
	//int culled = 0;
	//
	//std::queue<int> to_check;
	//std::queue<int> marked;
	//
	//for (int x = 0; x < params.points_x; ++x)
	//{
	//	for (int y = 0; y < params.points_y; ++y)
	//	{
	//		for (int z = 0; z < params.points_z; ++z, ++grid_loc)
	//		{
	//			limit = artifact_size;
	//
	//			to_check.push(grid_loc);
	//
	//			while (!to_check.empty() && limit > 0)
	//			{
	//				current = to_check.front();
	//				to_check.pop();
	//
	//				if (grid[current].mark_for_cull || grid[current].voxel_type != MeshingVoxelType::SOLID) {
	//					continue;
	//				}
	//
	//				grid[current].mark_for_cull = true;
	//				marked.push(current);
	//
	//				--limit;
	//
	//				int z_1 = (current / step_z) % params.points_z;
	//				int y_1 = (current / step_y) % params.points_y;
	//				int x_1 = (current / step_x) % params.points_x;
	//
	//				if (cull_artifacts_harsh)
	//				{
	//					if (x_1 > 0)
	//					{
	//						to_check.push((x_1 - 1) * step_x + y_1 * step_y + z_1 * step_z);
	//					}
	//					if (x_1 < params.points_x - 1)
	//					{
	//						to_check.push((x_1 + 1) * step_x + y_1 * step_y + z_1 * step_z);
	//					}
	//
	//					if (y_1 > 0)
	//					{
	//						to_check.push(x_1 * step_x + (y_1 - 1) * step_y + z_1 * step_z);
	//					}
	//					if (y_1 < params.points_y - 1)
	//					{
	//						to_check.push(x_1 * step_x + (y_1 + 1) * step_y + z_1 * step_z);
	//					}
	//
	//					if (z_1 > 0)
	//					{
	//						to_check.push(x_1 * step_x + y_1 * step_y + (z_1 - 1) * step_z);
	//					}
	//					if (z_1 < params.points_z - 1)
	//					{
	//						to_check.push(x_1 * step_x + y_1 * step_y + (z_1 + 1) * step_z);
	//					}
	//				}
	//				else
	//				{
	//					x_lower = std::max(0, x_1 - 1) * step_x;
	//					x_upper = std::min(params.points_x, x_1 + 2) * step_x;
	//
	//					y_lower = std::max(0, y_1 - 1) * step_y;
	//					y_upper = std::min(params.points_y, y_1 + 2) * step_y;
	//
	//					z_lower = std::max(0, z_1 - 1) * step_z;
	//					z_upper = std::min(params.points_z, z_1 + 2) * step_z;
	//
	//					for (int x_2 = x_lower; x_2 < x_upper; x_2 += step_x)
	//					{
	//						for (int y_2 = y_lower; y_2 < y_upper; y_2 += step_y)
	//						{
	//							for (int z_2 = z_lower; z_2 < z_upper; z_2 += step_z)
	//							{
	//								to_check.push(x_2 + y_2 + z_2);
	//							}
	//						}
	//					}
	//				}
	//			}
	//
	//			while (!to_check.empty()) { to_check.pop(); }
	//
	//			if (limit <= 0)
	//			{
	//				while (!marked.empty()) {
	//					marked.pop();
	//				}
	//			}
	//			else
	//			{
	//				culled += marked.size();
	//
	//				while (!marked.empty()) {
	//					current = marked.front();
	//
	//					grid[current].voxel_type = MeshingVoxelType::AIR;
	//					grid[current].value = 1.0f;
	//
	//					marked.pop();
	//				}
	//			}
	//		}
	//	}
	//}
	//
	//for (int x = 0; x < params.points_x; ++x)
	//{
	//	for (int y = 0; y < params.points_y; ++y)
	//	{
	//		for (int z = 0; z < params.points_z; ++z, ++grid_loc)
	//		{
	//			grid[current].mark_for_cull = false;
	//		}
	//	}
	//}
	//
	//std::cout << "Culled: " << culled << "/" << (params.points_x * params.points_y * params.points_z) << std::endl;
}

void MeshingVoxelGrid::LoadGridAsDoubleArray(double* double_stream)
{

}

bool MeshingVoxelGrid::SaveDenseGridToBinaryFile(std::string filename)
{
	std::ofstream savefile;

	savefile.open(filename, std::ios::binary);

	if (!savefile.is_open())
	{
		std::cout << "Couldn't open file" << std::endl;
		return false;
	}

	std::cout << "Saving file..." << std::endl;

	std::vector<std::string> data;

	int gridsize = params.points_x * params.points_y * params.points_z;

	savefile.write(reinterpret_cast<char*>(&params.points_x), sizeof(int));
	savefile.write(reinterpret_cast<char*>(&params.points_y), sizeof(int));
	savefile.write(reinterpret_cast<char*>(&params.points_z), sizeof(int));

	//savefile.write(reinterpret_cast<char*>(&params.points_x), sizeof(int));

	savefile.write(reinterpret_cast<char*>(&params.voxel_size), sizeof(double));

	savefile.write(reinterpret_cast<char*>(&params.center.x()), sizeof(double));
	savefile.write(reinterpret_cast<char*>(&params.center.y()), sizeof(double));
	savefile.write(reinterpret_cast<char*>(&params.center.z()), sizeof(double));

	for (int i = 0; i < gridsize; ++i)
	{
		double written_value = (grid[i].voxel_type == MeshingVoxelType::SOLID ? grid[i].value : -grid[i].value);

		char* to_write = reinterpret_cast<char*>(&written_value);

		//std::cout << i << "; " << *reinterpret_cast<double*>(to_write) << std::endl;

		savefile.write(to_write, sizeof(double));
	}

	savefile.close();

	std::cout << "Saved" << std::endl;

	return true;
}

bool MeshingVoxelGrid::SaveGridFourierToBinaryFile(std::string filename, int corner_dim_x, int corner_dim_y, int corner_dim_z)
{
	std::ofstream savefile;

	savefile.open(filename, std::ios::binary);

	if (!savefile.is_open())
	{
		std::cout << "Couldn't open file" << std::endl;
		return false;
	}

	if (!SaveGridFourierToBinaryFile(savefile, corner_dim_x, corner_dim_y, corner_dim_z))
	{
		return false;
	}

	return true;
}

bool MeshingVoxelGrid::SaveDenseGridToPlaintextFile(std::string filename)
{
	std::ofstream savefile;

	savefile.open(filename);

	if (!savefile.is_open())
	{
		std::cout << "Couldn't open file" << std::endl;
		return false;
	}

	std::cout << "Saving file..." << std::endl;

	std::vector<std::string> data;

	int gridsize = params.points_x * params.points_y * params.points_z;
	
	StrPlusDelim(std::to_string(params.points_x), "\n", savefile);
	StrPlusDelim(std::to_string(params.points_y), "\n", savefile);
	StrPlusDelim(std::to_string(params.points_z), "\n", savefile);

	StrPlusDelim(std::to_string(params.voxel_size), "\n", savefile);
	StrPlusDelim(std::to_string(params.center.x()), "\n", savefile);
	StrPlusDelim(std::to_string(params.center.y()), "\n", savefile);
	StrPlusDelim(std::to_string(params.center.z()), "\n", savefile);

	for (int i = 0; i < gridsize; ++i)
	{
		double written_value = (grid[i].voxel_type == MeshingVoxelType::SOLID ? grid[i].value : -grid[i].value);

		StrPlusDelim(std::to_string(written_value), "\n", savefile);
	}

	savefile.close();

	std::cout << "Saved" << std::endl;

	return false;
}

void MeshingVoxelGrid::StrPlusDelim(std::string str, std::string delim, std::ofstream &savefile)
{
	savefile.write(str.c_str(), str.size());
	savefile.write(delim.c_str(), delim.size());
}

bool MeshingVoxelGrid::ReadDenseGridFromBinaryFile(std::string filename)
{
	std::ifstream savefile;

	savefile.open(filename, std::ios::binary);

	if (!savefile.is_open())
	{
		std::cout << "Couldn't open file" << std::endl;
		return false;
	}

	std::cout << "Reading File..." << std::endl;

	MeshingVoxelParams newParams;

	std::vector<std::string> data;
	char buffer[sizeof(double)];

	savefile.read(buffer, sizeof(int));
	newParams.points_x = *reinterpret_cast<int*>(buffer);
	savefile.read(buffer, sizeof(int));
	newParams.points_y = *reinterpret_cast<int*>(buffer);
	savefile.read(buffer, sizeof(int));
	newParams.points_z = *reinterpret_cast<int*>(buffer);

	//savefile.read(buffer, sizeof(int));

	savefile.read(buffer, sizeof(double));
	newParams.voxel_size = *reinterpret_cast<double*>(buffer);

	savefile.read(buffer, sizeof(double));
	newParams.center.x() = *reinterpret_cast<double*>(buffer);
	savefile.read(buffer, sizeof(double));
	newParams.center.y() = *reinterpret_cast<double*>(buffer);
	savefile.read(buffer, sizeof(double));
	newParams.center.z() = *reinterpret_cast<double*>(buffer);

	SetData(newParams);

	int gridsize = params.points_x * params.points_y * params.points_z;

	

	for (int i = 0; i < gridsize; ++i)
	{
		savefile.read(buffer, sizeof(double));
		double val = *reinterpret_cast<double*>(buffer);

		//std::cout << i << "; " << val << std::endl;

		grid[i].value = abs(val);
		grid[i].voxel_type = (val > 0) ? MeshingVoxelType::SOLID : MeshingVoxelType::AIR;
	}

	savefile.close();

	std::cout << "Finished Reading" << std::endl;

	return false;
}

bool MeshingVoxelGrid::ReadGridFourierFromBinary(std::string filename)
{
	std::ifstream savefile;

	savefile.open(filename, std::ios::binary);

	if (!savefile.is_open())
	{
		return false;
	}

	if (!ReadGridFourierFromBinary(savefile))
	{
		return false;
	}

	return true;
}

bool MeshingVoxelGrid::ReadDenseGridFromPlaintextFile(std::string filename)
{
	//TODO

	return false;
}

bool MeshingVoxelGrid::SaveGridFourierToBinaryFile(std::ofstream& savefile, int corner_dim_x, int corner_dim_y, int corner_dim_z)
{
	bool result = SaveMVP(savefile);

	if (result)
	{
		result = result && SaveGridFourierBinaryWithoutMVP(savefile, corner_dim_x, corner_dim_y, corner_dim_z);
	}

	return result;
}

int MeshingVoxelGrid::SaveMVP(std::ofstream& savefile)
{
	int byte_count = 0;

	savefile.write(reinterpret_cast<char*>(&params.points_x), sizeof(int));
	savefile.write(reinterpret_cast<char*>(&params.points_y), sizeof(int));
	savefile.write(reinterpret_cast<char*>(&params.points_z), sizeof(int));

	byte_count += 3 * sizeof(int);

	savefile.write(reinterpret_cast<char*>(&params.voxel_size), sizeof(double));

	byte_count += sizeof(double);

	savefile.write(reinterpret_cast<char*>(&params.center.x()), sizeof(double));
	savefile.write(reinterpret_cast<char*>(&params.center.y()), sizeof(double));
	savefile.write(reinterpret_cast<char*>(&params.center.z()), sizeof(double));

	byte_count += 3 * sizeof(double);

	return byte_count;
}

int MeshingVoxelGrid::SaveGridFourierBinaryWithoutMVP(std::ofstream& savefile, int corner_dim_x, int corner_dim_y, int corner_dim_z)
{
	int byte_count = 0;

	//std::vector<std::string> data;

	int gridsize = params.points_x * params.points_y * params.points_z;

	savefile.write(reinterpret_cast<char*>(&corner_dim_x), sizeof(int));
	savefile.write(reinterpret_cast<char*>(&corner_dim_y), sizeof(int));
	savefile.write(reinterpret_cast<char*>(&corner_dim_z), sizeof(int));

	byte_count += 3 * sizeof(int);

	fftw_complex* signal = new fftw_complex[gridsize];
	fftw_complex* result = new fftw_complex[gridsize];

	for (int i = 0; i < gridsize; ++i)
	{
		signal[i][0] = (grid[i].voxel_type == MeshingVoxelType::SOLID ? grid[i].value : -grid[i].value);
		signal[i][1] = 0;
	}

	fftw_plan p = fftw_plan_dft_3d(params.points_x, params.points_y, params.points_z, signal, result, FFTW_FORWARD, FFTW_ESTIMATE);

	fftw_execute(p);

	//StandardFourierIteration(savefile, result, corner_dim_x, corner_dim_y, corner_dim_z);
	byte_count += CompactFourierIteration(savefile, result, corner_dim_x, corner_dim_y, corner_dim_z);

	//std::cout << "Saved" << std::endl;

	delete[] signal;
	delete[] result;

	return byte_count;
}

int MeshingVoxelGrid::SaveGridZipBinaryWithoutMVP(std::ofstream& savefile)
{
	int byte_count = 0;

	int gridsize = params.points_x * params.points_y * params.points_z;

	double new_val;
	double prev_val;

	int count_start = 0;
	int count = 0;

	for (int i = 0; i < gridsize; ++i)
	{
		new_val = (grid[i].voxel_type == MeshingVoxelType::SOLID ? grid[i].value : -grid[i].value);

		if (i == 0)
		{
			prev_val = new_val;
		}
		else if (new_val != prev_val)
		{
			savefile.write(reinterpret_cast<char*>(&count), sizeof(int));
			savefile.write(reinterpret_cast<char*>(&prev_val), sizeof(double));

			byte_count += sizeof(int);
			byte_count += sizeof(double);

			count_start = i;
			count = 0;

			prev_val = new_val;
		}

		++count;
	}

	savefile.write(reinterpret_cast<char*>(&count), sizeof(int));
	savefile.write(reinterpret_cast<char*>(&prev_val), sizeof(double));

	byte_count += sizeof(int);
	byte_count += sizeof(double);

	return byte_count;
}

int MeshingVoxelGrid::StandardFourierIteration(std::ofstream& savefile, fftw_complex* result, int corner_dim_x, int corner_dim_y, int corner_dim_z)
{
	int byte_count = 0;

	for (int x = 0; x < corner_dim_x; ++x)
	{
		for (int y = 0; y < corner_dim_y; ++y)
		{
			for (int z = 0; z < corner_dim_z; ++z)
			{
				int loc1 = GetArrayLoc(x, y, z);
				int loc2 = GetArrayLoc(x, y, params.points_z - 1 - z);
				int loc3 = GetArrayLoc(x, params.points_y - 1 - y, z);
				int loc4 = GetArrayLoc(x, params.points_y - 1 - y, params.points_z - 1 - z);
				int loc5 = GetArrayLoc(params.points_x - 1 - x, y, z);
				int loc6 = GetArrayLoc(params.points_x - 1 - x, y, params.points_z - 1 - z);
				int loc7 = GetArrayLoc(params.points_x - 1 - x, params.points_y - 1 - y, z);
				int loc8 = GetArrayLoc(params.points_x - 1 - x, params.points_y - 1 - y, params.points_z - 1 - z);

				char* to_write1 = reinterpret_cast<char*>(&result[loc1][0]);
				char* to_write2 = reinterpret_cast<char*>(&result[loc1][1]);
				char* to_write3 = reinterpret_cast<char*>(&result[loc2][0]);
				char* to_write4 = reinterpret_cast<char*>(&result[loc2][1]);
				char* to_write5 = reinterpret_cast<char*>(&result[loc3][0]);
				char* to_write6 = reinterpret_cast<char*>(&result[loc3][1]);
				char* to_write7 = reinterpret_cast<char*>(&result[loc4][0]);
				char* to_write8 = reinterpret_cast<char*>(&result[loc4][1]);
				char* to_write9 = reinterpret_cast<char*>(&result[loc5][0]);
				char* to_write10 = reinterpret_cast<char*>(&result[loc5][1]);
				char* to_write11 = reinterpret_cast<char*>(&result[loc6][0]);
				char* to_write12 = reinterpret_cast<char*>(&result[loc6][1]);
				char* to_write13 = reinterpret_cast<char*>(&result[loc7][0]);
				char* to_write14 = reinterpret_cast<char*>(&result[loc7][1]);
				char* to_write15 = reinterpret_cast<char*>(&result[loc8][0]);
				char* to_write16 = reinterpret_cast<char*>(&result[loc8][1]);

				savefile.write(to_write1, sizeof(double));
				savefile.write(to_write2, sizeof(double));
				savefile.write(to_write3, sizeof(double));
				savefile.write(to_write4, sizeof(double));
				savefile.write(to_write5, sizeof(double));
				savefile.write(to_write6, sizeof(double));
				savefile.write(to_write7, sizeof(double));
				savefile.write(to_write8, sizeof(double));
				savefile.write(to_write9, sizeof(double));
				savefile.write(to_write10, sizeof(double));
				savefile.write(to_write11, sizeof(double));
				savefile.write(to_write12, sizeof(double));
				savefile.write(to_write13, sizeof(double));
				savefile.write(to_write14, sizeof(double));
				savefile.write(to_write15, sizeof(double));
				savefile.write(to_write16, sizeof(double));
			}
		}
	}

	byte_count += 16 * corner_dim_x * corner_dim_y * corner_dim_z * sizeof(double);

	return byte_count;
}

int MeshingVoxelGrid::CompactFourierIteration(std::ofstream& savefile, fftw_complex* result, int corner_dim_x, int corner_dim_y, int corner_dim_z)
{
	int byte_count = 0;

	auto span = corner_dim_z * 2 * sizeof(double);

	int z_front = 0;
	int z_back = params.points_z - 1 - corner_dim_z;

	for (int x = 0; x < corner_dim_x; ++x)
	{
		for (int y = 0; y < corner_dim_y; ++y)
		{
			int loc1 = GetArrayLoc(x, y, z_front);
			int loc2 = GetArrayLoc(x, y, z_back);
			int loc3 = GetArrayLoc(x, params.points_y - 1 - y, z_front);
			int loc4 = GetArrayLoc(x, params.points_y - 1 - y, z_back);
			int loc5 = GetArrayLoc(params.points_x - 1 - x, y, z_front);
			int loc6 = GetArrayLoc(params.points_x - 1 - x, y, z_back);
			int loc7 = GetArrayLoc(params.points_x - 1 - x, params.points_y - 1 - y, z_front);
			int loc8 = GetArrayLoc(params.points_x - 1 - x, params.points_y - 1 - y, z_back);

			char* to_write1 = reinterpret_cast<char*>(&result[loc1][0]);
			char* to_write2 = reinterpret_cast<char*>(&result[loc2][0]);
			char* to_write3 = reinterpret_cast<char*>(&result[loc3][0]);
			char* to_write4 = reinterpret_cast<char*>(&result[loc4][0]);
			char* to_write5 = reinterpret_cast<char*>(&result[loc5][0]);
			char* to_write6 = reinterpret_cast<char*>(&result[loc6][0]);
			char* to_write7 = reinterpret_cast<char*>(&result[loc7][0]);
			char* to_write8 = reinterpret_cast<char*>(&result[loc8][0]);

			savefile.write(to_write1, span);
			savefile.write(to_write2, span);
			savefile.write(to_write3, span);
			savefile.write(to_write4, span);
			savefile.write(to_write5, span);
			savefile.write(to_write6, span);
			savefile.write(to_write7, span);
			savefile.write(to_write8, span);
		}
	}

	byte_count += 8 * corner_dim_x * corner_dim_y * span;

	return byte_count;
}

bool MeshingVoxelGrid::ReadGridFourierFromBinary(std::ifstream& savefile)
{
	char buffer[sizeof(double)];

	savefile.read(buffer, sizeof(int));
	int points_x = *reinterpret_cast<int*>(buffer);
	savefile.read(buffer, sizeof(int));
	int points_y = *reinterpret_cast<int*>(buffer);
	savefile.read(buffer, sizeof(int));
	int points_z = *reinterpret_cast<int*>(buffer);

	int gridsize = points_x * points_y * points_z;

	savefile.read(buffer, sizeof(double));
	double voxel_size = *reinterpret_cast<double*>(buffer);

	savefile.read(buffer, sizeof(double));
	double center_x = *reinterpret_cast<double*>(buffer);
	savefile.read(buffer, sizeof(double));
	double center_y = *reinterpret_cast<double*>(buffer);
	savefile.read(buffer, sizeof(double));
	double center_z = *reinterpret_cast<double*>(buffer);

	MeshingVoxelParams newParams;

	newParams.center = Eigen::Vector3d(center_x, center_y, center_z);
	newParams.points_x = points_x;
	newParams.points_y = points_y;
	newParams.points_z = points_z;
	newParams.voxel_size = voxel_size;

	SetData(newParams);

	savefile.read(buffer, sizeof(int));
	int corner_dim_x = *reinterpret_cast<int*>(buffer);
	savefile.read(buffer, sizeof(int));
	int corner_dim_y = *reinterpret_cast<int*>(buffer);
	savefile.read(buffer, sizeof(int));
	int corner_dim_z = *reinterpret_cast<int*>(buffer);

	fftw_complex* signal = new fftw_complex[gridsize];
	fftw_complex* result = new fftw_complex[gridsize];

	for (int x = 0; x < corner_dim_x; ++x)
	{
		for (int y = 0; y < corner_dim_y; ++y)
		{
			for (int z = 0; z < corner_dim_z; ++z)
			{
				//Each of the 8 corners

				int loc1 = ((x)*points_y + (y)) * points_z + (z);
				int loc2 = ((x)*points_y + (y)) * points_z + (points_z - 1 - z);
				int loc3 = ((x)*points_y + (points_y - 1 - y)) * points_z + (z);
				int loc4 = ((x)*points_y + (points_y - 1 - y)) * points_z + (points_z - 1 - z);
				int loc5 = ((points_x - 1 - x) * points_y + (y)) * points_z + (z);
				int loc6 = ((points_x - 1 - x) * points_y + (y)) * points_z + (points_z - 1 - z);
				int loc7 = ((points_x - 1 - x) * points_y + (points_y - 1 - y)) * points_z + (z);
				int loc8 = ((points_x - 1 - x) * points_y + (points_y - 1 - y)) * points_z + (points_z - 1 - z);

				savefile.read(buffer, sizeof(double));
				result[loc1][0] = *reinterpret_cast<double*>(buffer);
				savefile.read(buffer, sizeof(double));
				result[loc1][1] = *reinterpret_cast<double*>(buffer);

				savefile.read(buffer, sizeof(double));
				result[loc2][0] = *reinterpret_cast<double*>(buffer);
				savefile.read(buffer, sizeof(double));
				result[loc2][1] = *reinterpret_cast<double*>(buffer);

				savefile.read(buffer, sizeof(double));
				result[loc3][0] = *reinterpret_cast<double*>(buffer);
				savefile.read(buffer, sizeof(double));
				result[loc3][1] = *reinterpret_cast<double*>(buffer);

				savefile.read(buffer, sizeof(double));
				result[loc4][0] = *reinterpret_cast<double*>(buffer);
				savefile.read(buffer, sizeof(double));
				result[loc4][1] = *reinterpret_cast<double*>(buffer);

				savefile.read(buffer, sizeof(double));
				result[loc5][0] = *reinterpret_cast<double*>(buffer);
				savefile.read(buffer, sizeof(double));
				result[loc5][1] = *reinterpret_cast<double*>(buffer);

				savefile.read(buffer, sizeof(double));
				result[loc6][0] = *reinterpret_cast<double*>(buffer);
				savefile.read(buffer, sizeof(double));
				result[loc6][1] = *reinterpret_cast<double*>(buffer);

				savefile.read(buffer, sizeof(double));
				result[loc7][0] = *reinterpret_cast<double*>(buffer);
				savefile.read(buffer, sizeof(double));
				result[loc7][1] = *reinterpret_cast<double*>(buffer);

				savefile.read(buffer, sizeof(double));
				result[loc8][0] = *reinterpret_cast<double*>(buffer);
				savefile.read(buffer, sizeof(double));
				result[loc8][1] = *reinterpret_cast<double*>(buffer);
			}
		}
	}

	fftw_plan p = fftw_plan_dft_3d(points_x, points_y, points_z, result, signal, FFTW_BACKWARD, FFTW_ESTIMATE);

	fftw_execute(p);

	for (int i = 0; i < gridsize; ++i)
	{
		grid[i].value = abs(signal[i][0]);
		grid[i].voxel_type = (signal[i][0] > 0) ? MeshingVoxelType::SOLID : MeshingVoxelType::AIR;
	}

	delete[] signal;
	delete[] result;

	return false;
}
