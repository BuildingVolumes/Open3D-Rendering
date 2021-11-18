#include "NodeWrapper.h"
#include <iostream>
#include <fstream>
#include <filesystem>

void NodeWrapper::WriteOBJ(std::string filename, std::string filepath, open3d::geometry::TriangleMesh* mesh)
{
	std::ofstream writer;

	if (filepath != "")
	{
		std::filesystem::create_directories(filepath);

		writer.open(filepath + "/" + filename);
	}
	else
	{
		writer.open(filename);
	}

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

		writer << "f " <<
			(tri.x() + 1) << "/" << (tri.x() + 1) << "/" << (tri.x() + 1) << " " <<
			(tri.y() + 1) << "/" << (tri.y() + 1) << "/" << (tri.y() + 1) << " " <<
			(tri.z() + 1) << "/" << (tri.z() + 1) << "/" << (tri.z() + 1) << "\n";
	}

	writer.close();
}

NodeWrapper::NodeWrapper()
{
	cm = new CameraManager();

	vgd = new VoxelGridData();
}

NodeWrapper::~NodeWrapper()
{
	delete cm;

	delete vgd;
}

void NodeWrapper::PrintHelp() {
	using namespace open3d;

	PrintOpen3DVersion();
	// clang-format off
	utility::LogInfo("Commands:");	
	utility::LogInfo(">   --help");
	utility::LogInfo(">   Prints this help message in the console");
	utility::LogInfo("");
	utility::LogInfo(">   --LoadCamerasLivescan [string, root folder] [float, playback speed]");
	utility::LogInfo(">   Loads data taken from a Livescan output folder");
	utility::LogInfo("");
	utility::LogInfo(">   --LoadCamerasLivescanWithMattes [string, root folder] [float, playback speed]");
	utility::LogInfo(">   Loads data taken from a Livescan output folder that contains mattes");
	utility::LogInfo("");
	utility::LogInfo(">   --LoadCamerasStructure [string, root folder]");
	utility::LogInfo(">   Loads data where the details are specified by a .structure file");
	utility::LogInfo("");
	utility::LogInfo(">   --Unload");
	utility::LogInfo(">   Unloads data, must be called before calling any 'LoadCameras' command again");
	utility::LogInfo("");
	utility::LogInfo(">   --DisableCamera [int, cameraNum]");
	utility::LogInfo(">   Disables a camera - while disabled, camera will not write to voxel grids or textures");
	utility::LogInfo("");
	utility::LogInfo(">   --EnableCamera [int, cameraNum]");
	utility::LogInfo(">   Enables a disabled camera");
	utility::LogInfo("");
	utility::LogInfo(">   --EditVoxelGridData");
	utility::LogInfo(">   Edits voxel grid data depending on the following commands: ");
	utility::LogInfo(">   >   --blocks [int] -> controls the blocks in the grid (default 1000");
	utility::LogInfo(">   >   --depthScale [float] -> alters the depth scaling when cameras write to the grid (default 1000.f)");
	utility::LogInfo(">   >   --maxDepth [float] -> how far the camera extends into the voxel grid (default 3.f)");
	utility::LogInfo(">   >   --deviceCode [string] -> which device to use (default CPU:0)");
	utility::LogInfo(">   >   --sdfTrunc [float] -> grid will not show changes that are less significant than this number (default 0.04f)");
	utility::LogInfo(">   >   --voxel_size [float] -> the size of a single voxel (default 0.005859375f)");
	utility::LogInfo("");
	utility::LogInfo(">   --MakeObj [ulong, time] [string, filename] [string, filepath]");
	utility::LogInfo(">   Extracts an OBJ mesh from the current data at the provided time, and saves it as filename in filepath");
	utility::LogInfo("");
	utility::LogInfo(">   --TextureObj [string, .obj file] [ulong, time] [string, filename] [string, filepath]");
	utility::LogInfo(">   Textures a pre-existing OBJ file according to present data, then save it as filename in filepath");

	// clang-format on
	utility::LogInfo("");
}

void NodeWrapper::PerformOperations(int maxSpecs, char** specs)
{
	specsLength = maxSpecs;

	if (maxSpecs <= 1)
	{
		PrintHelp();
		return;
	}

	int currentSpec = 1;
	while (currentSpec < maxSpecs)
	{
		char* spec = specs[currentSpec];

		++currentSpec;

		if (spec == "--LoadCamerasLivescan")
		{
			currentSpec += LoadDataLivescan(specs, currentSpec, false);
		}
		else if (spec == "--LoadCamerasLivescanWithMattes")
		{
			currentSpec += LoadDataLivescan(specs, currentSpec, true);
		}
		else if (spec == "--LoadCamerasStructure")
		{
			currentSpec += LoadDataStructure(specs, currentSpec);
		}
		else if (spec == "--Unload")
		{
			currentSpec += Unload(specs, currentSpec);
		}
		else if (spec == "--EditVoxelGridData")
		{
			currentSpec += SetVoxelGridData(specs, currentSpec);
		}
		else if (spec == "--MakeObj")
		{
			currentSpec += MakeOBJ(specs, currentSpec);
		}
		else if (spec == "--TextureObj")
		{
			currentSpec += TextureOBJ(specs, currentSpec);
		}
		else if (spec == "--EnableCamera")
		{
			currentSpec += EnableCamera(specs, currentSpec, true);
		}
		else if (spec == "--DisableCamera")
		{
			currentSpec += EnableCamera(specs, currentSpec, false);
		}
		else if (spec == "--help")
		{
			PrintHelp();
		}
		else
		{
			std::cout << "Unknown argument: " << spec << std::endl;
		}
	}
	
}

int NodeWrapper::MakeOBJ(char** specs, int startingLoc)
{
	int argAmount = 3;

	if (specsLength <= startingLoc + argAmount)
	{
		std::cout << "Invalid argument amount" << std::endl;

		return argAmount;
	}

	auto obj = cm->GetMeshAtTimestamp(vgd, std::stoull(specs[startingLoc])).ToLegacyTriangleMesh();

	WriteOBJ(specs[startingLoc + 1], specs[startingLoc + 2], &obj);

	return argAmount;
}

int NodeWrapper::TextureOBJ(char** specs, int startingLoc)
{
	int argAmount = 4;

	if (specsLength <= startingLoc + argAmount)
	{
		std::cout << "Invalid argument amount" << std::endl;

		return argAmount;
	}

	open3d::geometry::TriangleMesh mesh;

	open3d::io::ReadTriangleMesh(specs[startingLoc], mesh);

	auto stitched_image = cm->CreateUVMapAndTextureAtTimestamp(&mesh, std::stoull(specs[startingLoc + 1]), false);

	std::string filename = specs[startingLoc + 2];
	std::string filepath = specs[startingLoc + 3];

	if (filepath == "")
	{
		open3d::io::WriteImageToPNG(filename, *stitched_image);
	}
	else
	{
		open3d::io::WriteImageToPNG(filepath + "/" + filename, *stitched_image);
	}

	return argAmount;
}

int NodeWrapper::LoadDataLivescan(char** specs, int startingLoc, bool useMattes)
{
	int argAmount = 4;

	if (specsLength <= startingLoc + argAmount)
	{
		std::cout << "Invalid argument amount" << std::endl;

		return argAmount;
	}

	cm->LoadTypeLivescan(specs[startingLoc], useMattes ? specs[startingLoc] : "", std::stof(specs[startingLoc + 1]));

	return argAmount;
}

int NodeWrapper::LoadDataStructure(char** specs, int startingLoc)
{
	int argAmount = 4;

	if (specsLength <= startingLoc + argAmount)
	{
		std::cout << "Invalid argument amount" << std::endl;

		return argAmount;
	}

	cm->LoadTypeStructure(specs[startingLoc], specs[startingLoc + 1]);

	return argAmount;
}

int NodeWrapper::Unload(char** specs, int startingLoc)
{
	cm->Unload();

	return 0;
}

int NodeWrapper::SetVoxelGridData(char** specs, int startingLoc)
{
	int currentSpec = startingLoc;

	while (true)
	{
		if (specsLength <= currentSpec + 2)
		{
			return currentSpec - startingLoc;
		}

		char* spec = specs[currentSpec];

		if (spec == "--blocks")
		{
			++currentSpec;

			vgd->blocks = std::stoi(specs[currentSpec]);
		}
		else if (spec == "--maxDepth")
		{
			++currentSpec;

			vgd->depth_max = std::stof(specs[currentSpec]);
		}
		else if (spec == "--depthScale")
		{
			++currentSpec;

			vgd->depth_scale = std::stof(specs[currentSpec]);
		}
		else if (spec == "--deviceCode")
		{
			++currentSpec;

			vgd->device_code = specs[currentSpec];
		}
		else if (spec == "--sdfTrunc")
		{
			++currentSpec;

			vgd->signed_distance_field_truncation = std::stof(specs[currentSpec]);
		}
		else if (spec == "--voxelSize")
		{
			++currentSpec;

			vgd->voxel_size = std::stof(specs[currentSpec]);
		}
		else
		{
			return currentSpec - startingLoc;
		}

		++currentSpec;
	}

	return 0;
}

int NodeWrapper::EnableCamera(char** specs, int startingLoc, bool enable)
{
	int argAmount = 1;

	if (specsLength <= startingLoc + argAmount)
	{
		std::cout << "Invalid argument amount" << std::endl;

		return argAmount;
	}

	cm->SetCameraEnabled(std::stoi(specs[startingLoc]), enable);

	return argAmount;
}
