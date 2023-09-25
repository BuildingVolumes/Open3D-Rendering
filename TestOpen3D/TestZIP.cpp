#include "TestZIP.h"

#include <iostream>
#include <filesystem>
#include "VoxelGridData.h"

std::shared_ptr<MKV_Rendering::CameraManager> TestZIP::LoadCameras(std::string main_path, std::vector<std::string> subpaths_rgbd, std::vector<std::string> subpaths_intrinsics, std::vector<std::string> subpaths_extrinsics)
{
	double capture_width = 1.2;
	int base_width = 201;
	int base_height = 401;

	MeshingVoxelParams params(capture_width / base_width, base_width, base_height, base_width, Eigen::Vector3d(0, 0, 0));

	//int edge_length = 100;
	//MeshingVoxelParams params(1.0 / (double)(edge_length - 1), edge_length, edge_length, edge_length, Eigen::Vector3d(0, 0, 0));

	auto p_tot = (params.points_x) * (params.points_y) * (params.points_z);

	auto cm = std::make_shared<MKV_Rendering::CameraManager>();

	for (int i = 0; i < subpaths_rgbd.size(); ++i)
	{
		cm->AddCameraLivescan(main_path + subpaths_rgbd[i], main_path + subpaths_intrinsics[i], main_path + subpaths_extrinsics[i],
			"Color_", "Depth_", "Matte_", 5.0);
	}

	return cm;
}

void TestZIP::run(int argc, char** argv)
{
	ZIP_Forward("TestingZip");
	ZIP_Backward("TestingZip");
}

void TestZIP::WriteOBJ(std::string filename, open3d::geometry::TriangleMesh* mesh)
{
	std::ofstream writer;

	writer.open(filename);

	for (int i = 0; i < mesh->vertices_.size(); ++i)
	{
		auto vert = mesh->vertices_[i];

		writer << "v " << vert.x() << " " << vert.y() << " " << vert.z() << "\n";
	}

	for (int i = 0; i < mesh->triangle_uvs_.size(); ++i)
	{
		auto uv = mesh->triangle_uvs_[i];

		writer << "vt " << uv.x() << " " << uv.y() << "\n";
	}

	for (int i = 0; i < mesh->vertex_normals_.size(); ++i)
	{
		auto norm = mesh->vertex_normals_[i];

		writer << "vn " << norm.x() << " " << norm.y() << " " << norm.z() << "\n";
	}

	for (int i = 0; i < mesh->triangles_.size(); ++i)
	{
		auto tri = mesh->triangles_[i];

		if (mesh->triangle_uvs_.size() > 0)
		{
			writer << "f " <<
				(tri.x() + 1) << "/" << (tri.x() + 1) << "/" << (tri.x() + 1) << " " <<
				(tri.y() + 1) << "/" << (tri.y() + 1) << "/" << (tri.y() + 1) << " " <<
				(tri.z() + 1) << "/" << (tri.z() + 1) << "/" << (tri.z() + 1) << "\n";
		}
		else
		{
			writer << "f " <<
				(tri.x() + 1) << "//" << (tri.x() + 1) << " " <<
				(tri.y() + 1) << "//" << (tri.y() + 1) << " " <<
				(tri.z() + 1) << "//" << (tri.z() + 1) << "\n";
		}
	}

	writer.close();
}

void TestZIP::ZIP_Forward(std::string ZIP_Name)
{
	double capture_width = 1.2;
	int base_width = 201;
	int base_height = 401;

	MeshingVoxelParams params(capture_width / base_width, base_width, base_height, base_width, Eigen::Vector3d(0, 0, 0));

	MKV_Rendering::VoxelGridData vgd;
	vgd.voxel_size = 9.f / 512.f;

	uint64_t timestamp = 21800000;

	SetPaths();

	auto cm = LoadCameras(path, rgbds, intrinsics, extrinsics);

	auto grid = cm->GetNewVoxelGridAtTimestamp(params, timestamp);

	auto mesh = grid->ExtractMesh();

	WriteOBJ(ZIP_Name + ".obj", &(*mesh));


	//system("tar.exe -h");

	auto start = std::chrono::high_resolution_clock::now();

	system(std::string("tar.exe -a -c -f " + ZIP_Name + ".zip " + ZIP_Name + ".obj").c_str());

	auto end = std::chrono::high_resolution_clock::now();

	std::chrono::duration<double> elapsed = end - start;

	std::cout << "Seconds elapsed: " << elapsed.count() << std::endl;
	//system(std::string("zip -r " + ZIP_Name + ".zip " + ZIP_Name).c_str());
}

void TestZIP::ZIP_Backward(std::string ZIP_Name)
{
	auto start = std::chrono::high_resolution_clock::now();

	system(std::string("tar.exe -x -f " + ZIP_Name + ".zip").c_str());

	auto end = std::chrono::high_resolution_clock::now();

	std::chrono::duration<double> elapsed = end - start;

	std::cout << "Seconds elapsed: " << elapsed.count() << std::endl;
}
