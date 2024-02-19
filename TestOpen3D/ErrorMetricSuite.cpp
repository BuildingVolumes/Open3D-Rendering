#include "ErrorMetricSuite.h"

#include "AdditionalUtilities.h"
#include <chrono>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/OBJ.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

void ErrorMetricSuite::TestHaudorff()
{
	std::string path_1 = "D:\\VsprojectsOnD\\Open3D\\TestOpen3D\\TestOpen3D\\TestOpen3D\\_AUXILIARY_TEST_DATA\\VolumetricSOAR\\AB-all\\AB-1punch";
	std::string path_2 = "D:\\VsprojectsOnD\\Open3D\\TestOpen3D\\TestOpen3D\\TestOpen3D\\_MeshDump\\_DecompressionDump";

	std::string path_3 = "D:\\VsprojectsOnD\\Open3D\\TestOpen3D\\TestOpen3D\\TestOpen3D\\_MeshDump\\_TriangulatedMeshes";

	auto obj_before = GetFilesWithTag(GetFiles(path_1), ".obj");
	auto obj_after = GetFilesWithTag(GetFiles(path_2), ".obj");

	std::vector<std::string> obj_before_triangles;

	for (int i = 0; i < obj_before.size(); ++i)
	{
		obj_before_triangles.push_back(path_3 + "\\Triangulated_" + GetNumberFixedLength(i, 6) + ".obj");
	}

	std::cout << "Triangulating..." << std::endl;

	pmr.TriangulateQuadOBJ(obj_before[0], obj_before_triangles[0]);

	std::cout << "Done triangulating" << std::endl;

	std::shared_ptr<open3d::geometry::TriangleMesh> mesh_before = std::make_shared<open3d::geometry::TriangleMesh>();
	std::shared_ptr<open3d::geometry::TriangleMesh> mesh_after = std::make_shared<open3d::geometry::TriangleMesh>();

	//open3d::io::ReadFileGeometryTypeOBJ()
	//open3d::io::readme

	//mesh_before = open3d::io::CreateMeshFromFile(obj_before[0]);
	//mesh_after = open3d::io::CreateMeshFromFile(obj_after[0]);

	//std::cout << "File type: " << open3d::io::ReadFileGeometryTypeOBJ(obj_before[0]) << std::endl;

	if (!open3d::io::ReadTriangleMesh(obj_before_triangles[0], *mesh_before))
	{
		std::cout << "Could not read " << obj_before_triangles[0] << std::endl;
		return;
	}
	if (!open3d::io::ReadTriangleMesh(obj_after[0], *mesh_after))
	{
		std::cout << "Could not read " << obj_after[0] << std::endl;
		return;
	}

	std::cout << "Stats for Before mesh:\n\tTris: " << mesh_before->triangles_.size() << "\n\tVerts: " << mesh_before->vertices_.size() << std::endl;
	std::cout << "Stats for After mesh:\n\tTris: " << mesh_after->triangles_.size() << "\n\tVerts: " << mesh_after->vertices_.size() << std::endl;

	std::chrono::steady_clock clock;

	std::cout << "Calculating Hausdorff... " << std::endl;

	auto timestamp = clock.now();

	auto hd_val = merm.HausdorffDistances(&(*mesh_before), &(*mesh_after));

	std::chrono::duration delta = clock.now() - timestamp;

	std::cout << "Hausdorff value: " << hd_val << ", achieved in " << delta.count() * 0.000000001 << " seconds." << std::endl;
}

void ErrorMetricSuite::TestPointToPlane()
{
	std::string path_1 = "D:\\VsprojectsOnD\\Open3D\\TestOpen3D\\TestOpen3D\\TestOpen3D\\_AUXILIARY_TEST_DATA\\VolumetricSOAR\\AB-all\\AB-1punch";
	std::string path_2 = "D:\\VsprojectsOnD\\Open3D\\TestOpen3D\\TestOpen3D\\TestOpen3D\\_MeshDump\\_DecompressionDump";

	std::string path_3 = "D:\\VsprojectsOnD\\Open3D\\TestOpen3D\\TestOpen3D\\TestOpen3D\\_MeshDump\\_TriangulatedMeshes";

	auto obj_before = GetFilesWithTag(GetFiles(path_1), ".obj");
	auto obj_after = GetFilesWithTag(GetFiles(path_2), ".obj");

	std::vector<std::string> obj_before_triangles;

	for (int i = 0; i < obj_before.size(); ++i)
	{
		obj_before_triangles.push_back(path_3 + "\\Triangulated_" + GetNumberFixedLength(i, 6) + ".obj");
	}

	std::cout << "Triangulating..." << std::endl;

	pmr.TriangulateQuadOBJ(obj_before[0], obj_before_triangles[0]);

	std::cout << "Done triangulating" << std::endl;

	std::shared_ptr<open3d::geometry::TriangleMesh> mesh_before = std::make_shared<open3d::geometry::TriangleMesh>();
	std::shared_ptr<open3d::geometry::TriangleMesh> mesh_after = std::make_shared<open3d::geometry::TriangleMesh>();

	if (!open3d::io::ReadTriangleMesh(obj_before_triangles[0], *mesh_before))
	{
		std::cout << "Could not read " << obj_before_triangles[0] << std::endl;
		return;
	}
	if (!open3d::io::ReadTriangleMesh(obj_after[0], *mesh_after))
	{
		std::cout << "Could not read " << obj_after[0] << std::endl;
		return;
	}

	std::cout << "Stats for Before mesh:\n\tTris: " << mesh_before->triangles_.size() << "\n\tVerts: " << mesh_before->vertices_.size() << std::endl;
	std::cout << "Stats for After mesh:\n\tTris: " << mesh_after->triangles_.size() << "\n\tVerts: " << mesh_after->vertices_.size() << std::endl;

	std::chrono::steady_clock clock;

	std::cout << "Calculating Point-To-Plane... " << std::endl;

	auto timestamp = clock.now();

	auto hd_val = merm.ComputerPointToPlane(&(*mesh_before), &(*mesh_after));
	//auto hd_val = merm.ComputerPointToPlane(&(*mesh_before), &(*mesh_before));

	std::chrono::duration delta = clock.now() - timestamp;

	std::cout << "Point-To-Plane value: " << hd_val << ", achieved in " << delta.count() * 0.000000001 << " seconds." << std::endl;
}

void ErrorMetricSuite::CGAL_TestMeshing()
{
	CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Kernel> sm;

	//CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(,)
}

void ErrorMetricSuite::run(int argc, char** argv)
{
	//TestHaudorff();

	TestPointToPlane();
}
