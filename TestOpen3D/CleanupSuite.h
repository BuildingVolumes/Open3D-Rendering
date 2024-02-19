#pragma once

#include "TestSuite.h"

#include "CameraManager.h"
#include "ShallowVoxelGrid.h"
#include "MeshErrorMetrics.h"
#include "DataMarshalling.h"

class CleanupSuite : public TestSuite
{
	//std::string to_cleanup = "D:/VsprojectsOnD/Open3D/TestOpen3D/TestOpen3D/TestOpen3D/_MeshDump/_TriangulatedMeshes";
	
	DataMarshalling dm;

	MKV_Rendering::VoxelGridData vgd;

	MeshingVoxelParams mvp;

	MeshErrorMetrics mem;

	std::string root_folder = "_TEST_DATA/july15-spinninghogue_0";
	std::string intrinsics_handle = "Intrinsics_Calib_";
	std::string extrinsics_handle = "Extrinsics_";
	std::string color_handle = "Color_";
	std::string depth_handle = "Depth_";
	std::string matte_handle = ".matte";

	std::string output_folder = "_MeshDump/_PiecewiseMeshes";
	//float FPS;

	MKV_Rendering::CameraManager cm;

	ShallowVoxelGrid svg;

	template<class T>
	void DrawObject(T& object_to_draw)
	{
		std::vector<std::shared_ptr<const open3d::geometry::Geometry>> to_draw;

		auto object_ptr = std::make_shared<T>(
			object_to_draw);

		to_draw.push_back(object_ptr);

		open3d::visualization::DrawGeometries(to_draw);
	}

	void ShrinkWrapMesh();

	void TestHoleRemoval();

	unsigned int RemoveHoles(CGAL::Surface_mesh<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3>* sm);

	void TestInhouseHoleRemoval(open3d::geometry::TriangleMesh* m);

	void TestSVG();

	void TestMeshCast();

	void TestBitShaver();

	std::shared_ptr<open3d::geometry::VoxelGrid> CreateFromTriangleMesh(
		const open3d::geometry::TriangleMesh& input,
		double voxel_size);

public:
	void run(int argc, char** argv);
};