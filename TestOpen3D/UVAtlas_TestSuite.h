#pragma once

#include "TestSuite.h"

#include "UVAtlas.h"

#include "VV_Mesh.h"
#include <string>

class UVAtlasTestSuite : public TestSuite
{
	VV_Mesh vv_m;

	std::string mesh_path = "D:/VsprojectsOnD/Open3D/TestOpen3D/TestOpen3D/TestOpen3D/_MeshDump/_TriangulatedMeshes/Triangulated_000000.obj";
public:
	void run(int argc, char** argv);
};