#pragma once

#include "TestSuite.h"

#include "MeshErrorMetrics.h"
#include "PolygonalMeshReader.h"

#include "open3d/geometry/TriangleMesh.h"
#include "open3d/geometry/TetraMesh.h"
#include "open3d/Open3D.h"

class ErrorMetricSuite : public TestSuite
{
	MeshErrorMetrics merm;

	PolygonalMeshReader pmr;

	void TestHaudorff();

	void TestPointToPlane();

	void CGAL_TestMeshing();
public:
	void run(int argc, char** argv);
};