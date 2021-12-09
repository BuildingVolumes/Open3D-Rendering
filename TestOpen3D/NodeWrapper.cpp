#include "NodeWrapper.h"
#include <iostream>
#include <fstream>
#include <filesystem>
#include "AdditionalUtilities.h"

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
			(tri.z() + 1) << "/" << (tri.z() + 1) << "/" << (tri.z() + 1);

		if (i < mesh->triangles_.size() - 1)
		{
			writer << "\n";
		}
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
	DebugLine("Commands:");	
	DebugLine(">   --help");
	DebugLine(">   Prints this help message in the console");
	DebugLine("");
	DebugLine(">   --emptyTXT [string, name]");
	DebugLine(">   Creates a TXT file");
	DebugLine("");
	DebugLine(">   --LoadCamerasLivescan [string, root folder] [float, playback speed]");
	DebugLine(">   Loads data taken from a Livescan output folder");
	DebugLine("");
	DebugLine(">   --LoadCamerasLivescanWithMattes [string, root folder] [float, playback speed]");
	DebugLine(">   Loads data taken from a Livescan output folder that contains mattes");
	DebugLine("");
	DebugLine(">   --LoadCamerasStructure [string, root folder]");
	DebugLine(">   Loads data where the details are specified by a .structure file");
	DebugLine("");
	DebugLine(">   --Unload");
	DebugLine(">   Unloads data, must be called before calling any 'LoadCameras' command again");
	DebugLine("");
	DebugLine(">   --DisableCamera [int, cameraNum]");
	DebugLine(">   Disables a camera - while disabled, camera will not write to voxel grids or textures");
	DebugLine("");
	DebugLine(">   --EnableCamera [int, cameraNum]");
	DebugLine(">   Enables a disabled camera");
	DebugLine("");
	DebugLine(">   --EditVoxelGridData");
	DebugLine(">   Edits voxel grid data depending on the following commands: ");
	DebugLine(">   >   --blocks [int] -> controls the blocks in the grid (default 1000");
	DebugLine(">   >   --depthScale [float] -> alters the depth scaling when cameras write to the grid (default 1000.f)");
	DebugLine(">   >   --maxDepth [float] -> how far the camera extends into the voxel grid (default 3.f)");
	DebugLine(">   >   --deviceCode [string] -> which device to use (default CPU:0)");
	DebugLine(">   >   --sdfTrunc [float] -> grid will not show changes that are less significant than this number (default 0.04f)");
	DebugLine(">   >   --voxel_size [float] -> the size of a single voxel (default 0.005859375f)");
	DebugLine("");
	DebugLine(">   --MakeObj [ulong, time] [string, filename] [string, filepath]");
	DebugLine(">   Extracts an OBJ mesh from the current data at the provided time, and saves it as filename in filepath");
	DebugLine("");
	DebugLine(">   --TextureObj [string, .obj file] [ulong, time] [string, filename] [string, filepath]");
	DebugLine(">   Textures a pre-existing OBJ file according to present data, then save it as filename in filepath");
	DebugLine("");
}

void NodeWrapper::PerformOperations(int maxSpecs, char** specs)
{
	pseudoSpecs.clear();

	for (int i = 0; i < maxSpecs; ++i)
	{
		pseudoSpecs.push_back(specs[i]);
	}

	//pseudoSpecs.push_back("TestOpen3D");
	//pseudoSpecs.push_back("--LoadCamerasLivescanWithMattes");
	//pseudoSpecs.push_back("_TEST_DATA/hogue_160");
	//pseudoSpecs.push_back("5.0");
	//pseudoSpecs.push_back("--MakeObj");
	//pseudoSpecs.push_back("0");
	//pseudoSpecs.push_back("NodesOBJ");
	//pseudoSpecs.push_back("TempFiles");
	//
	//maxSpecs = pseudoSpecs.size();

	//"TestOpen3D --LoadCamerasLivescanWithMattes ..\..\TestOpen3D\_TEST_DATA\hogue_160 5.0 --MakeObj 0 NodesOBJ TempFiles"

	specsLength = maxSpecs;

	if (maxSpecs <= 1)
	{
		PrintHelp();
		return;
	}

	try
	{
		int currentSpec = 1;
		while (currentSpec < maxSpecs)
		{
			std::string spec = pseudoSpecs[currentSpec];

			++currentSpec;

			if (spec == "--LoadCamerasLivescan")
			{
				currentSpec += LoadDataLivescan(currentSpec, false);
			}
			else if (spec == "--LoadCamerasLivescanWithMattes")
			{
				currentSpec += LoadDataLivescan(currentSpec, true);
			}
			else if (spec == "--LoadCamerasStructure")
			{
				currentSpec += LoadDataStructure(currentSpec);
			}
			else if (spec == "--Unload")
			{
				currentSpec += Unload(currentSpec);
			}
			else if (spec == "--EditVoxelGridData")
			{
				currentSpec += SetVoxelGridData(currentSpec);
			}
			else if (spec == "--MakeObj")
			{
				currentSpec += MakeOBJ(currentSpec);
			}
			else if (spec == "--TextureObj")
			{
				currentSpec += TextureOBJ(currentSpec);
			}
			else if (spec == "--EnableCamera")
			{
				currentSpec += EnableCamera(currentSpec, true);
			}
			else if (spec == "--DisableCamera")
			{
				currentSpec += EnableCamera(currentSpec, false);
			}
			else if (spec == "--help")
			{
				PrintHelp();
			}
			else if (spec == "--emptyTXT")
			{
				currentSpec += MakeEmptyTXT(currentSpec);
			}
			else
			{
				std::cout << "Unknown argument: " << spec << std::endl;
			}
		}
	}
	catch (std::exception &e)
	{
		WriteError(e, "ERRORS.txt");
	}
}

int NodeWrapper::MakeEmptyTXT(int currentSpec)
{
	int argAmount = 1;

	std::fstream emptyTXT;

	std::string name = pseudoSpecs[currentSpec];

	emptyTXT.open(name + ".txt", std::ios_base::out);

	if (!emptyTXT.is_open())
	{
		std::cout << "Couldn't make the file" << std::endl;
	}

	//std::vector<std::string> all_files;
	//std::vector<std::string> all_folders;

	//all_files = GetFiles(".");
	
	//open3d::utility::filesystem::ListFilesInDirectory("../..", all_files);
	
	//for (int i = 0; i < all_files.size(); ++i)
	//{
	//	std::cout << all_files[i] << std::endl;
	//}
	//
	//std::cout << std::filesystem::current_path() << std::endl;
	//
	//std::cout << "FINISHED" << std::endl;

	//emptyTXT.write(name.c_str(), name.length());
	emptyTXT.close();

	return argAmount;
}

void NodeWrapper::DebugLine(std::string text)
{
	std::cout << text << std::endl;
}

int NodeWrapper::MakeOBJ(int startingLoc)
{
	int argAmount = 3;

	if (specsLength < startingLoc + argAmount)
	{
		std::cout << "Invalid argument amount" << std::endl;

		return argAmount;
	}

	auto obj = cm->GetMeshAtTimestamp(vgd, std::stoull(pseudoSpecs[startingLoc])).ToLegacyTriangleMesh();

	WriteOBJ(pseudoSpecs[startingLoc + 1] + ".obj", pseudoSpecs[startingLoc + 2], &obj);

	return argAmount;
}

int NodeWrapper::TextureOBJ(int startingLoc)
{
	int argAmount = 4;

	if (specsLength < startingLoc + argAmount)
	{
		std::cout << "Invalid argument amount" << std::endl;

		return argAmount;
	}

	open3d::geometry::TriangleMesh mesh;

	open3d::io::ReadTriangleMesh(pseudoSpecs[startingLoc], mesh);

	auto stitched_image = cm->CreateUVMapAndTextureAtTimestamp(&mesh, std::stoull(pseudoSpecs[startingLoc + 1]), false);

	std::string filename = pseudoSpecs[startingLoc + 2];
	std::string filepath = pseudoSpecs[startingLoc + 3];

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

int NodeWrapper::LoadDataLivescan(int startingLoc, bool useMattes)
{
	int argAmount = 2;

	if (specsLength < startingLoc + argAmount)
	{
		std::cout << "Invalid argument amount" << std::endl;

		return argAmount;
	}

	cm->LoadTypeLivescan(pseudoSpecs[startingLoc], useMattes ? pseudoSpecs[startingLoc] : "", std::stof(pseudoSpecs[startingLoc + 1]));

	return argAmount;
}

int NodeWrapper::LoadDataStructure(int startingLoc)
{
	int argAmount = 2;

	if (specsLength < startingLoc + argAmount)
	{
		std::cout << "Invalid argument amount" << std::endl;

		return argAmount;
	}

	cm->LoadTypeStructure(pseudoSpecs[startingLoc], pseudoSpecs[startingLoc + 1]);

	return argAmount;
}

int NodeWrapper::Unload(int startingLoc)
{
	cm->Unload();

	return 0;
}

int NodeWrapper::SetVoxelGridData(int startingLoc)
{
	int currentSpec = startingLoc;

	while (true)
	{
		if (specsLength < currentSpec + 2)
		{
			return currentSpec - startingLoc;
		}

		std::string spec = pseudoSpecs[currentSpec];

		if (spec == "--blocks")
		{
			++currentSpec;

			vgd->blocks = std::stoi(pseudoSpecs[currentSpec]);
		}
		else if (spec == "--maxDepth")
		{
			++currentSpec;

			vgd->depth_max = std::stof(pseudoSpecs[currentSpec]);
		}
		else if (spec == "--depthScale")
		{
			++currentSpec;

			vgd->depth_scale = std::stof(pseudoSpecs[currentSpec]);
		}
		else if (spec == "--deviceCode")
		{
			++currentSpec;

			vgd->device_code = pseudoSpecs[currentSpec];
		}
		else if (spec == "--sdfTrunc")
		{
			++currentSpec;

			vgd->signed_distance_field_truncation = std::stof(pseudoSpecs[currentSpec]);
		}
		else if (spec == "--voxelSize")
		{
			++currentSpec;

			vgd->voxel_size = std::stof(pseudoSpecs[currentSpec]);
		}
		else
		{
			return currentSpec - startingLoc;
		}

		++currentSpec;
	}

	return 0;
}

int NodeWrapper::EnableCamera(int startingLoc, bool enable)
{
	int argAmount = 1;

	if (specsLength < startingLoc + argAmount)
	{
		std::cout << "Invalid argument amount" << std::endl;

		return argAmount;
	}

	cm->SetCameraEnabled(std::stoi(pseudoSpecs[startingLoc]), enable);

	return argAmount;
}

int NodeWrapper::CleanupMeshPoisson(int startingLoc)
{
	return 0;
}
