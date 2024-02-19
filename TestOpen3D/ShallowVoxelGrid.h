#pragma once

#include <vector>
#include <unordered_map>
#include <unordered_set>

#include "open3d/Open3D.h"

#include "MeshingVoxelGrid.h"

class ShallowVoxelGrid
{
	//std::unordered_set<Eigen::Vector3i> occupied;
	//std::unordered_map<Eigen::Vector3i, double[8]> value_map;

	std::unordered_map<size_t, double> edges_x_front;
	std::unordered_map<size_t, double> edges_x_back;
	std::unordered_map<size_t, double> edges_x_dist_front;
	std::unordered_map<size_t, double> edges_x_dist_back;

	std::unordered_map<size_t, double> edges_y_front;
	std::unordered_map<size_t, double> edges_y_back;
	std::unordered_map<size_t, double> edges_y_dist_front;
	std::unordered_map<size_t, double> edges_y_dist_back;

	std::unordered_map<size_t, double> edges_z_front;
	std::unordered_map<size_t, double> edges_z_back;
	std::unordered_map<size_t, double> edges_z_dist_front;
	std::unordered_map<size_t, double> edges_z_dist_back;


	int edge_count = 0;

	int value_size = 8 * sizeof(int);
	unsigned int max_value = -1;

	//std::vector<double> grid;

	Eigen::Vector3i dimensions;
	size_t grid_size;
	size_t step_x;
	size_t step_y;
	size_t step_z;

	Eigen::Vector3d center;
	Eigen::Vector3d first_corner;

	//int bits_per_data_point = 8;
	//unsigned int highest_bit_value = 255;

	double voxel_size = 0.02;

	double normal_epsilon = 0;// 0.00001;
	double epsilon_check = 0.000001;

	Eigen::Vector3d Max(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1);
	Eigen::Vector3d Min(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1);

public:
	void ClearGrid();

	size_t MultiplyIntoLong(Eigen::Vector3i &loc);

	size_t GetGridPosition(Eigen::Vector3i& loc);

	//void SetBitsPerDataPoint(int bits);

	void SetVoxelSize(double size);

	void SetGridDimensions(Eigen::Vector3i dims);

	bool CastMesh(open3d::geometry::TriangleMesh &mesh);

	std::shared_ptr<MeshingVoxelGrid> GetDenseVoxelGrid();

	//unsigned int GetValue(int loc);
	//
	//unsigned int GetValue(Eigen::Vector3i loc);

	//TODO: Remember the sorting of bytes approach!
};