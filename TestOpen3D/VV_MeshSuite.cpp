#include "VV_MeshSuite.h"

#include <iostream>

void VV_MeshSuite::TestLoadingAndSaving()
{
	std::string test_read = "D:/VsprojectsOnD/Open3D/TestOpen3D/TestOpen3D/TestOpen3D/_MeshDump/_TriangulatedMeshes/Triangulated_000000.obj";
	std::string test_write = "D:/VsprojectsOnD/Open3D/TestOpen3D/TestOpen3D/TestOpen3D/_MeshDump/Triangulated_Rewritten_000000.obj";

	test_mesh.ReadOBJ(test_read);

	std::cout << "VERTS: " << test_mesh.vertices.size() << std::endl;
	std::cout << "UVS: " << test_mesh.uvs.size() << std::endl;
	std::cout << "NORMALS: " << test_mesh.normals.size() << std::endl;

	std::cout << "TRI VERTS: " << test_mesh.triangles_vertices.size() << std::endl;
	std::cout << "TRI UVS: " << test_mesh.triangles_uvs.size() << std::endl;
	std::cout << "TRI NORMALS: " << test_mesh.triangles_normals.size() << std::endl;

	test_mesh.WriteOBJ(test_write);
}

void VV_MeshSuite::TestDecimation()
{
	double decimation_ratio = 0.1;

	std::string test_read = "D:/VsprojectsOnD/Open3D/TestOpen3D/TestOpen3D/TestOpen3D/_MeshDump/_TriangulatedMeshes/Triangulated_000000.obj";
	std::string test_write = "D:/VsprojectsOnD/Open3D/TestOpen3D/TestOpen3D/TestOpen3D/_MeshDump/Triangulated_Decimated_000000.obj";

	test_mesh.ReadOBJ(test_read);

	std::cout << "VERTS: " << test_mesh.vertices.size() << std::endl;
	std::cout << "UVS: " << test_mesh.uvs.size() << std::endl;
	std::cout << "NORMALS: " << test_mesh.normals.size() << std::endl;

	std::cout << "TRI VERTS: " << test_mesh.triangles_vertices.size() << std::endl;
	std::cout << "TRI UVS: " << test_mesh.triangles_uvs.size() << std::endl;
	std::cout << "TRI NORMALS: " << test_mesh.triangles_normals.size() << std::endl;

	auto decimated_mesh = test_mesh.DecimateEdges(decimation_ratio);

	std::cout << "DECIM VERTS: " << decimated_mesh->vertices.size() << std::endl;
	std::cout << "DECIM UVS: " << decimated_mesh->uvs.size() << std::endl;
	std::cout << "DECIM NORMALS: " << decimated_mesh->normals.size() << std::endl;

	std::cout << "DECIM TRI VERTS: " << decimated_mesh->triangles_vertices.size() << std::endl;
	std::cout << "DECIM TRI UVS: " << decimated_mesh->triangles_uvs.size() << std::endl;
	std::cout << "DECIM TRI NORMALS: " << decimated_mesh->triangles_normals.size() << std::endl;

	decimated_mesh->WriteOBJ(test_write);
}

void VV_MeshSuite::run(int argc, char** argv)
{
	//TestLoadingAndSaving();
	TestDecimation();
}
