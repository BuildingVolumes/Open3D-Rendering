#pragma once

#include "open3d/geometry/TriangleMesh.h"

#include <float.h>
#include <vector>
#include <unordered_map>

class MeshErrorMetrics
{
	double PointToPlaneOneWay(open3d::geometry::TriangleMesh* points, open3d::geometry::TriangleMesh* planes);

	double HausdorffOneWay(open3d::geometry::TriangleMesh* primary, open3d::geometry::TriangleMesh* secondary);

public:
	MeshErrorMetrics();

	double ComputerPointToPlane(open3d::geometry::TriangleMesh *m1, open3d::geometry::TriangleMesh* m2);

	double HausdorffDistances(open3d::geometry::TriangleMesh* m1, open3d::geometry::TriangleMesh* m2);

	std::vector<std::vector<int>> GetMeshOpenBorders(open3d::geometry::TriangleMesh* m1);
};