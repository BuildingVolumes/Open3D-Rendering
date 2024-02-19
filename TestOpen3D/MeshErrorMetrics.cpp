#include "MeshErrorMetrics.h"

#include <limits.h>

//#define DEBUG_ERROR_METRIC_PROGRESS 0
#define DEBUG_ERROR_METRIC_PROGRESS 1

double MeshErrorMetrics::PointToPlaneOneWay(open3d::geometry::TriangleMesh* points, open3d::geometry::TriangleMesh* planes)
{
#if DEBUG_ERROR_METRIC_PROGRESS
	int lines_per_debug = 100;
#endif // DEBUG_ERROR_METRIC_PROGRESS

	double tot = 0;

	int verts = points->vertices_.size();
	int tris = planes->triangles_.size();

	double checking_min;
	double line_dist;

	double sqr_min;

	for (int i = 0; i < tris; ++i)
	{
#if DEBUG_ERROR_METRIC_PROGRESS
		if (i % lines_per_debug == 0)
		{
			std::cout << "Triangle " << i << "..." << std::endl;
		}
#endif // DEBUG_ERROR_METRIC_PROGRESS

		sqr_min = DBL_MAX;

		auto tri = planes->triangles_[i];

		auto v0 = planes->vertices_[tri.x()];
		auto v1 = planes->vertices_[tri.y()];
		auto v2 = planes->vertices_[tri.z()];

		auto v01 = v0 - v1;
		auto v12 = v1 - v2;
		auto v20 = v2 - v0;

		if (v01.cross(v20).squaredNorm() <= 0)
		{
			continue;
		}

		auto tri_normal = -v01.cross(v20);

		for (int j = 0; j < verts; ++j)
		{
			checking_min = DBL_MAX;
			line_dist = DBL_MAX;

			//Get local distance from point to triangle
			auto pv0 = points->vertices_[j] - v0;
			auto pv1 = points->vertices_[j] - v1;
			auto pv2 = points->vertices_[j] - v2;

			//Get cross product to determine a consistent perpendicular direction
			auto c_v01_pv1 = v01.cross(pv1);
			auto c_v12_pv2 = v12.cross(pv2);
			auto c_v20_pv0 = v20.cross(pv0);

			//Dot product with triangle normal will determine scalar direction (is it on the 'left' or 'right' of the edge?)
			auto d_norm_1 = c_v01_pv1.dot(tri_normal);
			auto d_norm_2 = c_v12_pv2.dot(tri_normal);
			auto d_norm_0 = c_v20_pv0.dot(tri_normal);

			//If it is entirely on the left/right of all edges, it is inside the triangle's bounds
			bool check_front = (d_norm_1 <= 0) && (d_norm_2 <= 0) && (d_norm_0 <= 0);
			bool check_back = (d_norm_1 <= 0) && (d_norm_2 <= 0) && (d_norm_0 <= 0);

			if (check_front || check_back)
			{				
				//Do a simple point-to-plane distance
				checking_min = (pv0 - pv0.dot(tri_normal) / tri_normal.squaredNorm() * tri_normal).squaredNorm();
			}
			else
			{
				//Check if points exist near midpoints of lines
				auto l20_0_dot = pv0.dot(v20);
				auto l20_2_dot = -pv2.dot(v20);

				auto l12_2_dot = pv2.dot(v12);
				auto l12_1_dot = -pv1.dot(v12);

				auto l01_1_dot = pv1.dot(v01);
				auto l01_0_dot = -pv0.dot(v01);

				//If so, perform simple point-to-line distance with corresponding line. This must be done for all lines, as a point may exist near multiple line midpoints.
				if ((l20_0_dot > 0) && (l20_2_dot > 0))
				{
					line_dist = (pv0 - pv0.dot(v20) / v20.squaredNorm() * v20).squaredNorm();
					checking_min = (line_dist < checking_min) ? line_dist : checking_min;
				}
				if ((l12_2_dot > 0) && (l12_1_dot > 0))
				{
					line_dist = (pv2 - pv2.dot(v12) / v12.squaredNorm() * v12).squaredNorm();
					checking_min = (line_dist < checking_min) ? line_dist : checking_min;
				}
				if ((l01_1_dot > 0) && (l01_0_dot > 0))
				{
					line_dist = (pv1 - pv1.dot(v01) / v01.squaredNorm() * v01).squaredNorm();
					checking_min = (line_dist < checking_min) ? line_dist : checking_min;
				}
				
				//Finally check if nothing else is closer, choose the minimum of the point distances.
				checking_min = (pv0.squaredNorm() < checking_min) ? pv0.squaredNorm() : checking_min;
				checking_min = (pv1.squaredNorm() < checking_min) ? pv1.squaredNorm() : checking_min;
				checking_min = (pv2.squaredNorm() < checking_min) ? pv2.squaredNorm() : checking_min;
			}

			sqr_min = (checking_min < sqr_min) ? checking_min : sqr_min;
		}

		//We keep the square of the minimum as it does not affect greater-than operations for positive numbers. We sqrt it once we add the total.
		tot += sqrt(sqr_min);
	}

	return tot;
}

double MeshErrorMetrics::HausdorffOneWay(open3d::geometry::TriangleMesh* primary, open3d::geometry::TriangleMesh* secondary)
{
#if DEBUG_ERROR_METRIC_PROGRESS
	int lines_per_debug = 100;
#endif // DEBUG_ERROR_METRIC_PROGRESS

	double tot = 0;

	int prime_tri_count = primary->triangles_.size();
	int second_tri_count = secondary->triangles_.size();

	std::vector<double> point_dist;
	point_dist.resize(3);

	std::vector<double> dist_max;
	dist_max.resize(6);

	double new_min;
	double current_min;

	for (int i = 0; i < prime_tri_count; ++i)
	{
#if DEBUG_ERROR_METRIC_PROGRESS
		if (i % lines_per_debug == 0)
		{
			std::cout << "Line " << i << "..." << std::endl;
		}
#endif // DEBUG_ERROR_METRIC_PROGRESS

		current_min = DBL_MAX;

		auto p_t = primary->triangles_[i];

		auto p_0 = primary->vertices_[p_t.x()];
		auto p_1 = primary->vertices_[p_t.y()];
		auto p_2 = primary->vertices_[p_t.z()];

		for (int j = 0; j < second_tri_count; ++j)
		{
			auto s_t = secondary->triangles_[j];

			auto s_0 = secondary->vertices_[s_t.x()];
			auto s_1 = secondary->vertices_[s_t.y()];
			auto s_2 = secondary->vertices_[s_t.z()];


			point_dist[0] = (p_0 - s_0).squaredNorm();
			point_dist[1] = (p_0 - s_1).squaredNorm();
			point_dist[2] = (p_0 - s_2).squaredNorm();

			dist_max[0] = *std::min_element(point_dist.begin(), point_dist.end());

			point_dist[0] = (p_1 - s_0).squaredNorm();
			point_dist[1] = (p_1 - s_1).squaredNorm();
			point_dist[2] = (p_1 - s_2).squaredNorm();

			dist_max[1] = *std::min_element(point_dist.begin(), point_dist.end());

			point_dist[0] = (p_2 - s_0).squaredNorm();
			point_dist[1] = (p_2 - s_1).squaredNorm();
			point_dist[2] = (p_2 - s_2).squaredNorm();

			dist_max[2] = *std::min_element(point_dist.begin(), point_dist.end());

			new_min = *std::max_element(dist_max.begin(), dist_max.end());

			bool less = new_min < current_min;

			current_min = new_min * less + current_min * (!less);
		}

		tot += sqrt(current_min);
	}

	return tot;
}

MeshErrorMetrics::MeshErrorMetrics()
{
}

double MeshErrorMetrics::ComputerPointToPlane(open3d::geometry::TriangleMesh* m1, open3d::geometry::TriangleMesh* m2)
{
	return PointToPlaneOneWay(m1, m2) + PointToPlaneOneWay(m2, m1);
}

double MeshErrorMetrics::HausdorffDistances(open3d::geometry::TriangleMesh* m1, open3d::geometry::TriangleMesh* m2)
{
	return HausdorffOneWay(m1, m2) + HausdorffOneWay(m2, m1);
}

std::vector<std::vector<int>> MeshErrorMetrics::GetMeshOpenBorders(open3d::geometry::TriangleMesh* m1)
{
	std::vector<std::vector<int>> to_return;
	
	std::vector<std::vector<std::pair<int, int>>> follows;

	std::vector<std::pair<int, int>> unaccounted_edges;

	int degenerate_edge_count = 0;

	follows.resize(m1->vertices_.size());

	for (int i = 0; i < m1->triangles_.size(); ++i)
	{
		for (int j = 0; j < follows[m1->triangles_[i].x()].size(); ++j)
		{
			if (follows[m1->triangles_[i].x()][j].first == m1->triangles_[i].y())
			{
				++degenerate_edge_count;
			}
		}
		follows[m1->triangles_[i].x()].push_back(std::make_pair(m1->triangles_[i].y(), i));

		for (int j = 0; j < follows[m1->triangles_[i].y()].size(); ++j)
		{
			if (follows[m1->triangles_[i].y()][j].first == m1->triangles_[i].z())
			{
				++degenerate_edge_count;
			}
		}
		follows[m1->triangles_[i].y()].push_back(std::make_pair(m1->triangles_[i].z(), i));

		for (int j = 0; j < follows[m1->triangles_[i].z()].size(); ++j)
		{
			if (follows[m1->triangles_[i].z()][j].first == m1->triangles_[i].x())
			{
				++degenerate_edge_count;
			}
		}
		follows[m1->triangles_[i].z()].push_back(std::make_pair(m1->triangles_[i].x(), i));
	}

	for (int i = 0; i < follows.size(); ++i)
	{
		for (int j = 0; j < follows[i].size(); ++j)
		{
			auto edge1 = follows[i][j];
			bool alone = true;

			for (int k = 0; k < follows[edge1.first].size(); ++k)
			{
				auto edge2 = follows[edge1.first][k];

				if (edge2.first == i)
				{
					alone = false;
					continue;
				}
			}

			if (alone)
			{
				unaccounted_edges.push_back(edge1);
			}
		}
	}

	std::cout << "Loose edges: " << unaccounted_edges.size() << std::endl;

	std::cout << "Degenerate edges: " << degenerate_edge_count << std::endl;

	return to_return;
}
