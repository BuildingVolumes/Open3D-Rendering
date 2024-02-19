#include "ShallowVoxelGrid.h"

Eigen::Vector3d ShallowVoxelGrid::Max(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1)
{
	return Eigen::Vector3d(
		v0.x() > v1.x() ? v0.x() : v1.x(),
		v0.y() > v1.y() ? v0.y() : v1.y(),
		v0.z() > v1.z() ? v0.z() : v1.z());
}

Eigen::Vector3d ShallowVoxelGrid::Min(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1)
{
	return Eigen::Vector3d(
		v0.x() < v1.x() ? v0.x() : v1.x(),
		v0.y() < v1.y() ? v0.y() : v1.y(),
		v0.z() < v1.z() ? v0.z() : v1.z());
}

void ShallowVoxelGrid::ClearGrid()
{
	//grid.clear();

	edges_x_front.clear();
	edges_x_back.clear();
	edges_x_dist_front.clear();
	edges_x_dist_back.clear();

	edges_y_front.clear();
	edges_y_back.clear();
	edges_y_dist_front.clear();
	edges_y_dist_back.clear();

	edges_z_front.clear();
	edges_z_back.clear();
	edges_z_dist_front.clear();
	edges_z_dist_back.clear();

	edge_count = 0;
	//occupied.clear();
	//value_map.clear();
}

size_t ShallowVoxelGrid::MultiplyIntoLong(Eigen::Vector3i &loc)
{
	return (size_t)loc.x() * (size_t)loc.y() * (size_t)loc.z();
}

size_t ShallowVoxelGrid::GetGridPosition(Eigen::Vector3i& loc)
{
	return (size_t)loc.x() * step_x + (size_t)loc.y() * step_y + (size_t)loc.z() * step_z;
}

//void ShallowVoxelGrid::SetBitsPerDataPoint(int bits)
//{
//	bits_per_data_point = bits;
//
//	highest_bit_value = 1;
//	highest_bit_value <<= bits;
//	highest_bit_value -= 1;
//}

void ShallowVoxelGrid::SetVoxelSize(double size)
{
	voxel_size = size;
}

void ShallowVoxelGrid::SetGridDimensions(Eigen::Vector3i dims)
{
	dimensions = dims;
	grid_size = MultiplyIntoLong(dimensions);
	step_x = (size_t)dims.y() * (size_t)dims.z();
	step_y = (size_t)dims.z();
	step_z = 1;
}

bool ShallowVoxelGrid::CastMesh(open3d::geometry::TriangleMesh &mesh)
{
	if (mesh.triangles_.size() <= 0 || mesh.vertices_.size() <= 0)
	{
		std::cout << "Error when casting mesh to SDF: No triangles/vertices in mesh!" << std::endl;
		return false;
	}
	
	if (voxel_size <= 0)
	{
		std::cout << "Error when casting mesh to SDF: Invalid voxel size!" << std::endl;
		return false;
	}
	
	if (dimensions.x() <= 0 || dimensions.y() <= 0 || dimensions.z() <= 0)
	{
		std::cout << "Error when casting mesh to SDF: Invalid dimensions!" << std::endl;
		return false;
	}
	
	//if (bits_per_data_point <= 0 || bits_per_data_point > value_size)
	//{
	//	std::cout << "Error when casting mesh to SDF: Bits per data point must be between 1 and " + std::to_string(value_size) + "!" << std::endl;
	//	return false;
	//}
	
	ClearGrid();

	//grid.resize(grid_size, -1);

	//edges_x_front.resize(vec_size, -1);
	//edges_x_back.resize(vec_size, -1);
	//edges_y_front.resize(vec_size, -1);
	//edges_y_back.resize(vec_size, -1);
	//edges_z_front.resize(vec_size, -1);
	//edges_z_back.resize(vec_size, -1);
	
	//int grid_size = dimensions.x() * dimensions.y() * dimensions.z();
	//grid.resize(grid_size);
	
	//int quant_sized = grid_size / value_size + (grid_size % value_size != 0);
	
	//for (int i = 0; i < bits_per_data_point; ++i)
	//{
	//	grid.push_back(std::vector<unsigned int>());
	//	grid[i].resize(quant_sized);
	//}
	
	auto mesh_max = mesh.GetMaxBound();
	auto mesh_min = mesh.GetMinBound();
	
	//auto scaled_max = mesh_max / voxel_size;
	//auto scaled_min = mesh_min / voxel_size;
	
	//dimensions.x() = (ceil(scaled_max.x()) - floor(scaled_min.x()));
	//dimensions.y() = (ceil(scaled_max.y()) - floor(scaled_min.y()));
	//dimensions.z() = (ceil(scaled_max.z()) - floor(scaled_min.z()));
	
	center = (mesh_max + mesh_min) * 0.5;
	first_corner = center - Eigen::Vector3d(dimensions.x() - 1, dimensions.y() - 1, dimensions.z() - 1) * (0.5 * voxel_size);
	
	std::cout << "Mesh center: " << center.transpose() << std::endl;

	Eigen::Vector3d max_from_tri;
	Eigen::Vector3d min_from_tri;
	Eigen::Vector3i max_tri_bounds;
	Eigen::Vector3i min_tri_bounds;
	
	Eigen::Vector3i* tri;
	
	Eigen::Vector3d p0;
	Eigen::Vector3d p1;
	Eigen::Vector3d p2;
	
	Eigen::Vector3d p01;
	Eigen::Vector3d p12;
	Eigen::Vector3d p20;
	
	Eigen::Vector3d tri_center;
	Eigen::Vector3d from_center;

	Eigen::Vector3d normal;
	Eigen::Vector3d normalized_normal;

	double triple_prod;
	
	Eigen::Vector3d ray_loc;
	
	double cross_0;
	double cross_1;
	double cross_2;
	
	double dist;
	
	double planar_center_dist_sqr;

	double v0;
	double v1;
	
	double val_abs;
	double comp_abs;
	
	Eigen::Vector3i loc0;
	size_t loc0_scalar;
	Eigen::Vector3i loc1;
	size_t loc1_scalar;
	
	bool check_front;
	bool check_back;
	bool normal_forward;

	for (int i = 0; i < mesh.triangles_.size(); ++i)
	{
		tri = &mesh.triangles_[i];
	
		//Set up all the relevant point data for later
		p0 = mesh.vertices_[tri->x()] - first_corner;
		p1 = mesh.vertices_[tri->y()] - first_corner;
		p2 = mesh.vertices_[tri->z()] - first_corner;
	
		tri_center = (p0 + p1 + p2) / 3;

		p01 = p0 - p1;
		p12 = p1 - p2;
		p20 = p2 - p0;
	
		normal = -p01.cross(p20);
	
		normalized_normal = normal.normalized();

		triple_prod = p0.dot(normal);
	
		max_from_tri = Max(p0, Max(p1, p2)) / voxel_size;
		min_from_tri = Min(p0, Min(p1, p2)) / voxel_size;
	
		//Calculate the bounds that the triangle spans on the grid
		max_tri_bounds = Eigen::Vector3i(ceil(max_from_tri.x()), ceil(max_from_tri.y()), ceil(max_from_tri.z()));
		min_tri_bounds = Eigen::Vector3i(floor(min_from_tri.x()), floor(min_from_tri.y()), floor(min_from_tri.z()));
	
		max_tri_bounds = Eigen::Vector3i(
			(max_tri_bounds.x() <= (dimensions.x() - 1)) ? max_tri_bounds.x() : (dimensions.x() - 1),
			(max_tri_bounds.y() <= (dimensions.y() - 1)) ? max_tri_bounds.y() : (dimensions.y() - 1),
			(max_tri_bounds.z() <= (dimensions.z() - 1)) ? max_tri_bounds.z() : (dimensions.z() - 1)
			);
	
		min_tri_bounds = Eigen::Vector3i(
			(min_tri_bounds.x() >= 0) ? min_tri_bounds.x() : 0,
			(min_tri_bounds.y() >= 0) ? min_tri_bounds.y() : 0,
			(min_tri_bounds.z() >= 0) ? min_tri_bounds.z() : 0
			);
	
		//Cast rays into the triangle on XY, XZ, and YZ. If the triangle is out of bounds, then the loops simply won't run.
	
		if (abs(normalized_normal.z()) > normal_epsilon)
		{
			normal_forward = (normalized_normal.z() <= 0);

			for (int x = min_tri_bounds.x(); x <= max_tri_bounds.x(); ++x)
			{
				for (int y = min_tri_bounds.y(); y <= max_tri_bounds.y(); ++y)
				{
					loc0.x() = x;
					loc1.x() = x;
					loc0.y() = y;
					loc1.y() = y;

					ray_loc = Eigen::Vector3d(x * voxel_size, y * voxel_size, 0);

					dist = (triple_prod - normal.x() * ray_loc.x() - normal.y() * ray_loc.y()) / normal.z();

					dist /= voxel_size;

					if ((dist >= (dimensions.z() - 1)) || (dist < 0))
					{
						continue;
					}

					from_center = ray_loc - tri_center;
					planar_center_dist_sqr = from_center.x() * from_center.x() + from_center.y() * from_center.y();

					//Simplified cross products
					cross_0 = p01.x() * (ray_loc.y() - p1.y()) - p01.y() * (ray_loc.x() - p1.x());
					cross_1 = p12.x() * (ray_loc.y() - p2.y()) - p12.y() * (ray_loc.x() - p2.x());
					cross_2 = p20.x() * (ray_loc.y() - p0.y()) - p20.y() * (ray_loc.x() - p0.x());

					//Check if the point resides inside the triangle's bounds
					check_back = (cross_0 >= 0) && (cross_1 >= 0) && (cross_2 >= 0);
					check_front = (cross_0 <= 0) && (cross_1 <= 0) && (cross_2 <= 0);

					loc0.z() = floor(dist);
					loc1.z() = loc0.z() + 1;

					loc0_scalar = GetGridPosition(loc0);
					loc1_scalar = GetGridPosition(loc1);

					v0 = dist - floor(dist);
					v0 = ((v0 <= 0) ? epsilon_check : v0);
					v1 = v0 - 1;
					v1 = ((v1 >= 0) ? -epsilon_check : v1);


					if (check_front || check_back)
					{
						if (edges_z_front.find(loc0_scalar) == edges_z_front.end())
						{
							++edge_count;

							edges_z_dist_front[loc0_scalar] = voxel_size;
							edges_z_dist_back[loc0_scalar] = voxel_size;
							edges_z_front[loc0_scalar] = 1;
							edges_z_back[loc1_scalar] = -1;
						}

						if (normal_forward)
						{
							v0 = -v0;
							v1 = -v1;
						}

						val_abs = abs(edges_z_front[loc0_scalar]);
						comp_abs = abs(v0);

						edges_z_front[loc0_scalar] = (val_abs < comp_abs ? edges_z_front[loc0_scalar] : v0);

						val_abs = abs(edges_z_back[loc1_scalar]);
						comp_abs = abs(v1);

						edges_z_back[loc1_scalar] = (val_abs < comp_abs ? edges_z_back[loc1_scalar] : v1);
					}
					else if (planar_center_dist_sqr < voxel_size * voxel_size)
					{

					}
				}
			}
		}
	
		if (abs(normalized_normal.y()) > normal_epsilon)
		{
			normal_forward = (normalized_normal.y() <= 0);

			for (int z = min_tri_bounds.z(); z <= max_tri_bounds.z(); ++z)
			{
				for (int x = min_tri_bounds.x(); x <= max_tri_bounds.x(); ++x)
				{
					loc0.z() = z;
					loc1.z() = z;
					loc0.x() = x;
					loc1.x() = x;

					ray_loc = Eigen::Vector3d(x * voxel_size, 0, z * voxel_size);

					dist = (triple_prod - normal.z() * ray_loc.z() - normal.x() * ray_loc.x()) / normal.y();

					dist /= voxel_size;

					if ((dist >= (dimensions.y() - 1)) || (dist < 0))
					{
						continue;
					}

					from_center = ray_loc - tri_center;
					planar_center_dist_sqr = from_center.z() * from_center.z() + from_center.x() * from_center.x();

					//Simplified cross products
					cross_0 = p01.z() * (ray_loc.x() - p1.x()) - p01.x() * (ray_loc.z() - p1.z());
					cross_1 = p12.z() * (ray_loc.x() - p2.x()) - p12.x() * (ray_loc.z() - p2.z());
					cross_2 = p20.z() * (ray_loc.x() - p0.x()) - p20.x() * (ray_loc.z() - p0.z());

					//Check if the point resides inside the triangle's bounds
					check_back = (cross_0 >= 0) && (cross_1 >= 0) && (cross_2 >= 0);
					check_front = (cross_0 <= 0) && (cross_1 <= 0) && (cross_2 <= 0);

					loc0.y() = floor(dist);
					loc1.y() = loc0.y() + 1;

					loc0_scalar = GetGridPosition(loc0);
					loc1_scalar = GetGridPosition(loc1);

					v0 = dist - floor(dist);
					v0 = ((v0 <= 0) ? epsilon_check : v0);
					v1 = v0 - 1;
					v1 = ((v1 >= 0) ? -epsilon_check : v1);


					if (check_front || check_back)
					{
						if (edges_y_front.find(loc0_scalar) == edges_y_front.end())
						{
							++edge_count;

							edges_y_dist_front[loc0_scalar] = voxel_size;
							edges_y_dist_back[loc0_scalar] = voxel_size;
							edges_y_front[loc0_scalar] = 1;
							edges_y_back[loc1_scalar] = -1;
						}

						if (normal_forward)
						{
							v0 = -v0;
							v1 = -v1;
						}

						val_abs = abs(edges_y_front[loc0_scalar]);
						comp_abs = abs(v0);

						edges_y_front[loc0_scalar] = (val_abs < comp_abs ? edges_y_front[loc0_scalar] : v0);

						val_abs = abs(edges_y_back[loc1_scalar]);
						comp_abs = abs(v1);

						edges_y_back[loc1_scalar] = (val_abs < comp_abs ? edges_y_back[loc1_scalar] : v1);
					}
					else if (planar_center_dist_sqr < voxel_size * voxel_size)
					{

					}
				}
			}
		}
	
		if (abs(normalized_normal.x()) > normal_epsilon)
		{
			normal_forward = (normalized_normal.x() <= 0);

			for (int y = min_tri_bounds.y(); y <= max_tri_bounds.y(); ++y)
			{
				for (int z = min_tri_bounds.z(); z <= max_tri_bounds.z(); ++z)
				{
					loc0.y() = y;
					loc1.y() = y;
					loc0.z() = z;
					loc1.z() = z;

					ray_loc = Eigen::Vector3d(0, y * voxel_size, z * voxel_size);

					dist = (triple_prod - normal.y() * ray_loc.y() - normal.z() * ray_loc.z()) / normal.x();

					dist /= voxel_size;

					if ((dist >= (dimensions.x() - 1)) || (dist < 0))
					{
						continue;
					}

					from_center = ray_loc - tri_center;
					planar_center_dist_sqr = from_center.y() * from_center.y() + from_center.z() * from_center.z();

					//Simplified cross products
					cross_0 = p01.y() * (ray_loc.z() - p1.z()) - p01.z() * (ray_loc.y() - p1.y());
					cross_1 = p12.y() * (ray_loc.z() - p2.z()) - p12.z() * (ray_loc.y() - p2.y());
					cross_2 = p20.y() * (ray_loc.z() - p0.z()) - p20.z() * (ray_loc.y() - p0.y());

					//Check if the point resides inside the triangle's bounds
					check_back = (cross_0 >= 0) && (cross_1 >= 0) && (cross_2 >= 0);
					check_front = (cross_0 <= 0) && (cross_1 <= 0) && (cross_2 <= 0);

					loc0.x() = floor(dist);
					loc1.x() = loc0.x() + 1;

					loc0_scalar = GetGridPosition(loc0);
					loc1_scalar = GetGridPosition(loc1);

					v0 = dist - floor(dist);
					v0 = ((v0 <= 0) ? epsilon_check : v0);
					v1 = v0 - 1;
					v1 = ((v1 >= 0) ? -epsilon_check : v1);


					if (check_front || check_back)
					{
						if (edges_x_front.find(loc0_scalar) == edges_x_front.end())
						{
							++edge_count;

							edges_x_dist_front[loc0_scalar] = voxel_size;
							edges_x_dist_back[loc0_scalar] = voxel_size;
							edges_x_front[loc0_scalar] = 1;
							edges_x_back[loc1_scalar] = -1;
						}

						if (normal_forward)
						{
							v0 = -v0;
							v1 = -v1;
						}

						val_abs = abs(edges_x_front[loc0_scalar]);
						comp_abs = abs(v0);

						edges_x_front[loc0_scalar] = (val_abs < comp_abs ? edges_x_front[loc0_scalar] : v0);

						val_abs = abs(edges_x_back[loc1_scalar]);
						comp_abs = abs(v1);

						edges_x_back[loc1_scalar] = (val_abs < comp_abs ? edges_x_back[loc1_scalar] : v1);
					}
					else if (planar_center_dist_sqr < voxel_size * voxel_size)
					{

					}
				}
			}
		}
	}
	
	std::cout << "Recorded edges: " << edge_count << std::endl;
	
	return true;
}

std::shared_ptr<MeshingVoxelGrid> ShallowVoxelGrid::GetDenseVoxelGrid()
{
	auto to_return = std::make_shared<MeshingVoxelGrid>();

	MeshingVoxelParams mvp;

	mvp.center = center;
	mvp.points_x = dimensions.x();
	mvp.points_y = dimensions.y();
	mvp.points_z = dimensions.z();
	mvp.voxel_size = voxel_size;

	to_return->SetData(mvp);

	double val_min;
	double val_sum;
	int sum_tallied;
	size_t key = 0;

	double set_value;
	std::vector<size_t> to_set;

	for (int x = 0; x < dimensions.x(); ++x)
	{
		for (int y = 0; y < dimensions.y(); ++y)
		{
			set_value = -1;

			for (int z = 0; z < dimensions.z(); ++z, ++key)
			{
				val_sum = 0;
				sum_tallied = 0;
				val_min = 1;

				if (edges_x_front.find(key) != edges_x_front.end())
				{
					//if (abs(edges_x_front[key]) < val_min)
					//{
					//	val_min = edges_x_front[key];
					//}
					++sum_tallied;
					val_sum += edges_x_front[key];
				}
				if (edges_x_back.find(key) != edges_x_back.end())
				{
					//if (abs(edges_x_back[key]) < val_min)
					//{
					//	val_min = edges_x_back[key];
					//}
					++sum_tallied;
					val_sum += edges_x_back[key];
				}

				if (edges_y_front.find(key) != edges_y_front.end())
				{
					//if (abs(edges_y_front[key]) < val_min)
					//{
					//	val_min = edges_y_front[key];
					//}
					++sum_tallied;
					val_sum += edges_y_front[key];
				}
				if (edges_y_back.find(key) != edges_y_back.end())
				{
					//if (abs(edges_y_back[key]) < val_min)
					//{
					//	val_min = edges_y_back[key];
					//}
					++sum_tallied;
					val_sum += edges_y_back[key];
				}

				if (edges_z_front.find(key) != edges_z_front.end())
				{
					//if (abs(edges_z_front[key]) < val_min)
					//{
					//	val_min = edges_z_front[key];
					//}
					++sum_tallied;
					val_sum += edges_z_front[key];
				}
				if (edges_z_back.find(key) != edges_z_back.end())
				{
					//if (abs(edges_z_back[key]) < val_min)
					//{
					//	val_min = edges_z_back[key];
					//}
					++sum_tallied;
					val_sum += edges_z_back[key];
				}

				if (sum_tallied > 0)
				{
					val_sum /= sum_tallied;
					//val_sum = val_min;
					set_value = (val_sum >= 0) ? 1 : -1;

					//if (val_sum == 0)
					//{
					//	std::cout << "ZERO SUM! " << std::endl;
					//}

					to_return->SetGridValueKillNegatives(key, val_sum);

					for (int i = 0; i < to_set.size(); ++i)
					{
						to_return->SetGridValueKillNegatives(to_set[i], set_value);
					}

					to_set.clear();
				}
				else
				{
					to_set.push_back(key);
				}
			}
			
			//set_value = -1;

			for (int i = 0; i < to_set.size(); ++i)
			{
				to_return->SetGridValueKillNegatives(to_set[i], set_value);
			}

			to_set.clear();
		}
	}

	return to_return;
}
