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

NodeWrapper::NodeWrapper()
{
	vgd = new VoxelGridData();

	edm = new EncDec::EncodeDecodeManager();

	cm = new CameraManager();
}

NodeWrapper::~NodeWrapper()
{
	delete cm;

	delete edm;

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
	DebugLine(">   --LoadCamerasLivescan");
	DebugLine(">   Initializes a camera manager to take livescan cameras");
	DebugLine("");
	DebugLine(">   --AddCamera [string, rootFolder] [string, intrinsicsFile] [string, extrinsicsFile] [string, colorTag] [string, depthTag] [string, matteTag]");
	DebugLine(">   Adds a single camera's worth of data to the manager.");
	DebugLine(">   Root folder encompasses all relevant files for the camera.");
	DebugLine(">   Intrinsics/Extrinsics is the file name and path of the intrinsics/extrinsics file");
	DebugLine(">   Tags indicate what keywords to look for when searching for color/depth/matte images");
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
	DebugLine(">   --MakeObj [ulong, time] [string, filename]");
	DebugLine(">   Extracts an OBJ mesh from the current data at the provided time, and saves it as filename");
	DebugLine("");
	DebugLine(">   --TextureObj [string, .obj file] [ulong, time] [string, filename]");
	DebugLine(">   Textures a pre-existing OBJ file according to present data, then save it as filename");
	DebugLine("");
	DebugLine(">   --EncodeOBJ [string, .obj file] [int, channels] [int, bytes per channel] [string, .png filename]");
	DebugLine(">   Encodes an OBJ into a PNG file");
	DebugLine("");
	DebugLine(">   --DecodeOBJ [string, .png file] [string, .png filename]");
	DebugLine(">   Extracts an OBJ from a PNG file that was encoded through the EncodeOBJ function");
	DebugLine("");
	DebugLine(">   --MortonEncodeOBJ [string, .obj file] [int, channels] [int, bytes per channel] [string, .png filename]");
	DebugLine(">   Encodes an OBJ into a PNG file");
	DebugLine("");
	DebugLine(">   --MortonDecodeOBJ [string, .png file] [string, .png filename]");
	DebugLine(">   Extracts an OBJ from a PNG file that was encoded through the MortonEncodeOBJ function");
	DebugLine("");
}

void NodeWrapper::PerformOperations(int maxSpecs, char** specs)
{
	pseudoSpecs.clear();

	for (int i = 0; i < maxSpecs; ++i)
	{
		pseudoSpecs.push_back(specs[i]);
	}

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
				currentSpec += LoadDataLivescan(currentSpec);
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
			else if (spec == "--AddCamera")
			{ 
				currentSpec += AddCameraLivescan(currentSpec);
			}
			else if (spec == "--help")
			{
				PrintHelp();
			}
			else if (spec == "--emptyTXT")
			{
				currentSpec += MakeEmptyTXT(currentSpec);
			}
			else if (spec == "--EncodeOBJ")
			{
				currentSpec += EncodeMesh(currentSpec);
			}
			else if (spec == "--DecodeOBJ")
			{
				currentSpec += DecodeMesh(currentSpec);
			}
			else if (spec == "--MortonEncodeOBJ")
			{
				currentSpec += MortonEncodeMesh(currentSpec);
			}
			else if (spec == "--MortonDecodeOBJ")
			{
				currentSpec += MortonDecodeMesh(currentSpec);
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

	//system("pause");
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
	int argAmount = 2;

	if (specsLength < startingLoc + argAmount)
	{
		std::cout << "Invalid argument amount" << std::endl;

		return argAmount;
	}

	auto obj = cm->GetMeshAtTimestamp(vgd, std::stoull(pseudoSpecs[startingLoc])).ToLegacyTriangleMesh();

	open3d::io::WriteTriangleMeshToOBJ(pseudoSpecs[startingLoc + 1], obj, true, false, true, false, true, false);

	return argAmount;
}

int NodeWrapper::TextureOBJ(int startingLoc)
{
	int argAmount = 3;

	if (specsLength < startingLoc + argAmount)
	{
		std::cout << "Invalid argument amount" << std::endl;

		return argAmount;
	}

	open3d::geometry::TriangleMesh mesh;

	open3d::io::ReadTriangleMesh(pseudoSpecs[startingLoc], mesh);

	auto stitched_image = cm->CreateUVMapAndTextureAtTimestamp(&mesh, std::stoull(pseudoSpecs[startingLoc + 1]), false);

	std::string filename = pseudoSpecs[startingLoc + 2];

	open3d::io::WriteImageToPNG(filename, *stitched_image);
	open3d::io::WriteTriangleMeshToOBJ(pseudoSpecs[startingLoc], mesh, true, false, true, false, true, false);

	return argAmount;
}

int NodeWrapper::LoadDataLivescan(int startingLoc)
{
	int argAmount = 0;

	if (specsLength < startingLoc + argAmount)
	{
		std::cout << "Invalid argument amount" << std::endl;

		return argAmount;
	}

	cm->LoadTypeLivescan();

	return argAmount;
}

int NodeWrapper::AddCameraLivescan(int startingLoc)
{
	int argAmount = 6;

	if (specsLength < startingLoc + argAmount)
	{
		std::cout << "Invalid argument amount" << std::endl;

		return argAmount;
	}

	std::string root_folder = pseudoSpecs[startingLoc];
	
	std::string intrinsics = pseudoSpecs[startingLoc + 1];
	std::string extrinsics = pseudoSpecs[startingLoc + 2];

	std::string color_handle = pseudoSpecs[startingLoc + 3];
	std::string depth_handle = pseudoSpecs[startingLoc + 4];
	std::string matte_handle = pseudoSpecs[startingLoc + 5];

	cm->AddCameraLivescan(root_folder, intrinsics, extrinsics, color_handle, depth_handle, matte_handle, 5.0);

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

int NodeWrapper::EncodeMesh(int startingLoc)
{
	int argAmount = 4;

	std::string obj_file = pseudoSpecs[startingLoc];
	
	int image_channels = std::stoi(pseudoSpecs[startingLoc + 1]);
	int bytes_per_channel = std::stoi(pseudoSpecs[startingLoc + 2]);

	std::string output_file_name = pseudoSpecs[startingLoc + 3];

	auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();

	open3d::io::ReadTriangleMesh(obj_file, *mesh);

	auto image = edm->DefaultEncodeOBJ(mesh, image_channels, bytes_per_channel);

	open3d::io::WriteImageToPNG(output_file_name, *image);

	return argAmount;
}

int NodeWrapper::DecodeMesh(int startingLoc)
{
	int argAmount = 2;

	std::string png_file = pseudoSpecs[startingLoc];
	std::string output_file_name = pseudoSpecs[startingLoc + 1];

	auto image = std::make_shared<open3d::geometry::Image>();

	open3d::io::ReadImageFromPNG(png_file, *image);

	auto mesh = edm->DefaultDecodeOBJ(image);

	WriteOBJ(output_file_name, "", &(*mesh));

	return argAmount;
}

int NodeWrapper::MortonEncodeMesh(int startingLoc)
{
	int argAmount = 4;

	std::string obj_file = pseudoSpecs[startingLoc];

	int image_channels = std::stoi(pseudoSpecs[startingLoc + 1]);
	int bytes_per_channel = std::stoi(pseudoSpecs[startingLoc + 2]);

	std::string output_file_name = pseudoSpecs[startingLoc + 3];

	auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();

	open3d::io::ReadTriangleMesh(pseudoSpecs[startingLoc], *mesh);

	auto image = edm->MortonEncodeOBJ(mesh, image_channels, bytes_per_channel);

	open3d::io::WriteImageToPNG(output_file_name, *image);

	return argAmount;
}

int NodeWrapper::MortonDecodeMesh(int startingLoc)
{
	int argAmount = 2;

	std::string png_file = pseudoSpecs[startingLoc];
	std::string output_file_name = pseudoSpecs[startingLoc + 1];

	auto image = std::make_shared<open3d::geometry::Image>();

	open3d::io::ReadImageFromPNG(png_file, *image);

	auto mesh = edm->MortonDecodeOBJ(image);

	WriteOBJ(output_file_name, "", &(*mesh));

	return argAmount;
}
