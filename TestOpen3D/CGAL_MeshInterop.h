#pragma once

#include "CameraManager.h"

#include "DataMarshalling.h"

class CGAL_MeshInterop
{
	DataMarshalling dm;

public:
	void Compress(open3d::geometry::TriangleMesh* mesh, double simplification_ratio);
};