#include "MeshingVoxelGrid.h"

MeshingVoxelGrid::MeshingVoxelGrid(double voxel_size, int voxels_x, int voxels_y, int voxels_z, Eigen::Vector3d center)
{
	this->voxel_size = voxel_size;

	size_x = voxels_x;
	size_y = voxels_y;
	size_z = voxels_z;

	grid = new SingleVoxel[size_x * size_y * size_z];

	double loc_x = center.x() - voxel_size * 0.5 * (double)(size_x - 1);
	double loc_y = center.y() - voxel_size * 0.5 * (double)(size_y - 1);
	double loc_z = center.z() - voxel_size * 0.5 * (double)(size_z - 1);

	std::cout << "Lower left bound: " << loc_x << ", " << loc_y << ", " << loc_z << std::endl;

	std::cout << "Center: " << center.x() << ", " << center.y() << ", " << center.z() << std::endl;

	int grid_loc = 0;

	double inc_loc_x = loc_x;
	double inc_loc_y = loc_y;
	double inc_loc_z = loc_z;

	for (int x = 0; x < size_x; ++x, inc_loc_x += voxel_size)
	{
		inc_loc_y = loc_y;

		for (int y = 0; y < size_y; ++y, inc_loc_y += voxel_size)
		{
			inc_loc_z = loc_z;

			for (int z = 0; z < size_z; ++z, ++grid_loc, inc_loc_z += voxel_size)
			{
				grid[grid_loc].position = Eigen::Vector3d(inc_loc_x, inc_loc_y, inc_loc_z);
				grid[grid_loc].color = Eigen::Vector3d((double)x / (double)size_x,(double)y / (double)size_y, (double)z / (double)size_z);
			}
		}
	}

	inc_loc_x -= voxel_size;
	inc_loc_y -= voxel_size;
	inc_loc_z -= voxel_size;

	std::cout << "Upper right bound: " << inc_loc_x << ", " << inc_loc_y << ", " << inc_loc_z << std::endl;
}

MeshingVoxelGrid::~MeshingVoxelGrid()
{
	delete[] grid;
}

void MeshingVoxelGrid::AddImage(open3d::geometry::Image& color, open3d::geometry::Image& depth, Eigen::Matrix4d extrinsics, Eigen::Matrix3d intrinsics)
{
	auto depth_float = depth.ConvertDepthToFloatImage();

	int culled = 0;
	int solid = 0;
	int air = 0;

	int grid_loc = 0;

	Eigen::Matrix3d rotation = extrinsics.block<3, 3>(0, 0);
	Eigen::Vector3d position = extrinsics.block<3, 1>(0, 3);

	Eigen::Matrix3d rotation_inv = rotation.inverse();

	Eigen::Matrix3d intrinsic_inv = intrinsics.inverse();

	for (int x = 0; x < size_x; ++x)
	{
		for (int y = 0; y < size_y; ++y)
		{
			for (int z = 0; z < size_z; ++z, ++grid_loc)
			{
				auto voxel = &grid[grid_loc];

				Eigen::Vector3d uvz = intrinsics *
					(rotation * voxel->position + position);

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
				
				auto color_r = *color.PointerAt<uint8_t>(int_x, int_y, 0);
				auto color_g = *color.PointerAt<uint8_t>(int_x, int_y, 1);
				auto color_b = *color.PointerAt<uint8_t>(int_x, int_y, 2);

				//voxel->color = Eigen::Vector3d((double)color_r / 255.0, (double)color_g / 255.0, (double)color_b / 255.0);

				Eigen::Vector3d pixel_position = rotation_inv * (intrinsic_inv * Eigen::Vector3d(uvz.x(), uvz.y(), pixel_depth) - position);

				Eigen::Vector3d dist = (pixel_position - voxel->position);
				double mag = std::min(sqrt(dist.dot(dist)) / voxel_size, 1.0);

				//AIR
				if (pixel_depth > uvz.z() || pixel_depth == 0)
				{

					if (voxel->voxel_type == MeshingVoxelType::SOLID)
					{
						voxel->voxel_type = MeshingVoxelType::AIR;
						voxel->value = std::min(mag, 1.0);
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
						voxel->value = std::min(voxel->value, mag);
						++solid;
					}
					else if (voxel->voxel_type == MeshingVoxelType::AIR)
					{

					}
					else
					{
						voxel->voxel_type = MeshingVoxelType::SOLID;
						voxel->value = std::min(mag, 1.0);
						
						++solid;
					}
				}
			}
		}
	}

	std::cout << "culled voxels: " << culled << "/" << (size_x * size_y * size_z) << std::endl;
	std::cout << "solid voxels: " << solid << "/" << (size_x * size_y * size_z) << std::endl;
	std::cout << "air voxels: " << air << "/" << (size_x * size_y * size_z) << std::endl;
}

std::shared_ptr<open3d::geometry::TriangleMesh> MeshingVoxelGrid::ExtractMesh()
{
	auto to_return = std::make_shared<open3d::geometry::TriangleMesh>();

	int grid_loc = 0;
	int solid_voxels = 0;

	for (int x = 0; x < size_x; ++x)
	{
		for (int y = 0; y < size_y; ++y)
		{
			for (int z = 0; z < size_z; ++z, ++grid_loc)
			{
				solid_voxels += (grid[grid_loc].voxel_type == MeshingVoxelType::SOLID);
			}
		}
	}

	std::cout << "Solid voxels total: " << solid_voxels << "/" << (size_x * size_y * size_z) << std::endl;

	int index_count = 0;

	int corners[8];
	MeshingVoxelEdge edges[12];

	int step_x = size_z * size_y;
	int step_y = size_z;
	int step_z = 1;

	SingleVoxel corner_voxels[8];

	int no_triangles = 0;

	for (int x = 0; x < size_x - 1; ++x)
	{
		for (int y = 0; y < size_y - 1; ++y)
		{
			for (int z = 0; z < size_z - 1; ++z)
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

				corner_voxels[0] = grid[corners[0]];
				corner_voxels[4] = grid[corners[4]];
				corner_voxels[2] = grid[corners[2]];
				corner_voxels[6] = grid[corners[6]];
				corner_voxels[1] = grid[corners[1]];
				corner_voxels[5] = grid[corners[5]];
				corner_voxels[3] = grid[corners[3]];
				corner_voxels[7] = grid[corners[7]];

				//Testing which edges will be produced
				int index = 0;

				index |= 1 *	(corner_voxels[0].voxel_type == MeshingVoxelType::SOLID);
				index |= 2 *	(corner_voxels[1].voxel_type == MeshingVoxelType::SOLID);
				index |= 8 *	(corner_voxels[2].voxel_type == MeshingVoxelType::SOLID);
				index |= 4 *	(corner_voxels[3].voxel_type == MeshingVoxelType::SOLID);
				index |= 16 *	(corner_voxels[4].voxel_type == MeshingVoxelType::SOLID);
				index |= 32 *	(corner_voxels[5].voxel_type == MeshingVoxelType::SOLID);
				index |= 128 *	(corner_voxels[6].voxel_type == MeshingVoxelType::SOLID);
				index |= 64 *	(corner_voxels[7].voxel_type == MeshingVoxelType::SOLID);

				index = 255 - index;

				if (edge_table[index] == 0)
				{
					++no_triangles;
					continue;
				}

				//Interpolating edges
				if ((edge_table[index] & 1) > 0)
					edges[0] = LerpCorner(corner_voxels, 0, 1);
				if ((edge_table[index] & 2) > 0)
					edges[1] = LerpCorner(corner_voxels, 1, 3);
				if ((edge_table[index] & 4) > 0)
					edges[2] = LerpCorner(corner_voxels, 3, 2);
				if ((edge_table[index] & 8) > 0)
					edges[3] = LerpCorner(corner_voxels, 2, 0);
				if ((edge_table[index] & 16) > 0)
					edges[4] = LerpCorner(corner_voxels, 4, 5);
				if ((edge_table[index] & 32) > 0)
					edges[5] = LerpCorner(corner_voxels, 5, 7);
				if ((edge_table[index] & 64) > 0)
					edges[6] = LerpCorner(corner_voxels, 7, 6);
				if ((edge_table[index] & 128) > 0)
					edges[7] = LerpCorner(corner_voxels, 6, 4);
				if ((edge_table[index] & 256) > 0)
					edges[8] = LerpCorner(corner_voxels, 0, 4);
				if ((edge_table[index] & 512) > 0)
					edges[9] = LerpCorner(corner_voxels, 1, 5);
				if ((edge_table[index] & 1024) > 0)
					edges[10] = LerpCorner(corner_voxels, 3, 7);
				if ((edge_table[index] & 2048) > 0)
					edges[11] = LerpCorner(corner_voxels, 2, 6);

				auto tri_table_seg = tri_table[index];

				//Adding triangles to the mesh
				for (int i = 0; tri_table_seg[i] != -1; i += 3, index_count += 3)
				{
					to_return->vertices_.push_back(edges[tri_table_seg[i]].position);
					to_return->vertex_colors_.push_back(edges[tri_table_seg[i]].color);

					to_return->vertices_.push_back(edges[tri_table_seg[i + 1]].position);
					to_return->vertex_colors_.push_back(edges[tri_table_seg[i + 1]].color);

					to_return->vertices_.push_back(edges[tri_table_seg[i + 2]].position);
					to_return->vertex_colors_.push_back(edges[tri_table_seg[i + 2]].color);

					to_return->triangles_.push_back(
						Eigen::Vector3i(index_count, index_count + 1, index_count + 2)
					);
				}
			}
		}
	}

	std::cout << "Mesh vertices: " << to_return->vertices_.size() << std::endl;
	std::cout << "Voxels without triangles: " << no_triangles << "/" << ((size_x - 1) * (size_y - 1) * (size_z - 1)) << std::endl;

	return to_return;
}

MeshingVoxelEdge MeshingVoxelGrid::LerpCorner(SingleVoxel* voxel_array, int elem1, int elem2)
{
	double t = voxel_array[elem2].value / (voxel_array[elem1].value + voxel_array[elem2].value);

	return MeshingVoxelEdge(
		voxel_array[elem1].position * (1.0 - t) + voxel_array[elem2].position * t,
		voxel_array[elem1].color * (1.0 - t) + voxel_array[elem2].color * t
		);
}
