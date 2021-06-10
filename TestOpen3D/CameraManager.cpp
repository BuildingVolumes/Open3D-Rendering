#include "CameraManager.h"
#include "AdditionalUtilities.h"
#include "ErrorLogger.h"

#include "MKV_Data.h"
#include "Image_Data.h"

#include <fstream>

using namespace MKV_Rendering;

void MKV_Rendering::CameraManager::CauseError(bool cause_abort)
{
	bool bad_thing_happened = true;

	if (bad_thing_happened)
	{
		ErrorLogger::LOG_ERROR("DONT WORRY! This error is suppsoed to happen here!", cause_abort);
	}
}

void MKV_Rendering::CameraManager::LoadStructure(std::string structure_path, std::map<std::string, std::string> *data)
{
	std::fstream structure_file;
	std::vector<std::string> lines;
	std::string parser;

	structure_file.open(structure_path);

	if (!structure_file.is_open())
	{
		ErrorLogger::LOG_ERROR("No suitable structure file found at " + structure_path + "!", true);
	}

	while (std::getline(structure_file, parser))
	{
		if (parser != "")
		{
			lines.push_back(parser);

			parser = "";
		}
	}

	for (auto line : lines)
	{
		std::vector<std::string> name_and_value;
		SplitString(line, name_and_value, ' ');
		(*data)[name_and_value[0]] = name_and_value[1];
	}

	structure_file.close();
}

CameraManager::CameraManager(std::string root_folder, std::string structure_file_name)
{
	std::vector<std::string> all_folders = GetDirectories(root_folder);
	//open3d::utility::filesystem::ListFilesInDirectory(root_folder, all_folders);

	std::fstream structure_file;
	std::vector<std::string> lines;

	std::map<std::string, std::string> camera_structure;
	std::string parser;

	for (auto _folder : all_folders)
	{
		std::string structure_path = _folder + "/" + structure_file_name;

		ErrorLogger::EXECUTE("Load File Structure", this, &CameraManager::LoadStructure, structure_path, &camera_structure);

		std::cout << "Camera specifics at " << _folder << ": " << std::endl;
		for (auto _pair : camera_structure)
		{
			std::cout << "\t" << _pair.first << ": '" << _pair.second <<  "'" << std::endl;
		}

		std::string c_type = camera_structure["Type"];

		if (c_type == "mkv")
		{
			camera_data.push_back(new MKV_Data(
				_folder, 
				camera_structure["MKV_File"], 
				camera_structure["Calibration_File"]
			));
		}
		else if (c_type == "image")
		{
			camera_data.push_back(new Image_Data(
				_folder, 
				camera_structure["Color"], 
				camera_structure["Depth"], 
				camera_structure["Intrinsics_Json"],
				camera_structure["Calibration_File"],
				camera_structure["FPS"]
			));
		}
		else
		{
			ErrorLogger::LOG_ERROR("Unrecognized file format, " + c_type + "!");
		}

		camera_structure.clear();

		std::cout << "Finished initializing camera!\n" << std::endl;
	}

	if (camera_data.size() == 0)
	{
		ErrorLogger::LOG_ERROR(
			"No data files present!", true
		);
	}
}

CameraManager::~CameraManager()
{
	while (!camera_data.empty())
	{
		if (camera_data.back() != nullptr)
		{
			delete camera_data.back();
		}

		camera_data.pop_back();
	}
}

open3d::t::geometry::TriangleMesh MKV_Rendering::CameraManager::GetMesh(VoxelGridData* data)
{
	open3d::core::Device device(data->device_code);

	open3d::t::geometry::TSDFVoxelGrid voxel_grid(

		{
			{"tsdf", open3d::core::Dtype::Float32},
			{"weight", open3d::core::Dtype::UInt16},
			{"color", open3d::core::Dtype::UInt16}
		},

		data->voxel_size, data->signed_distance_field_truncation,
		16, data->blocks, device
	);

	for (auto cam : camera_data)
	{
		ErrorLogger::EXECUTE("Pack Frame into Voxel Grid", cam, &Abstract_Data::PackIntoVoxelGrid, &voxel_grid, data);
	}

	return voxel_grid.ExtractSurfaceMesh(0.0f);
}

bool MKV_Rendering::CameraManager::CycleAllCamerasForward()
{
	bool success = true;

	for (auto cam : camera_data)
	{
		success = success && ErrorLogger::EXECUTE("Cycle Camera Forward", cam, &Abstract_Data::CycleCaptureForwards);
	}

	return success;
}

bool MKV_Rendering::CameraManager::CycleAllCamerasBackward()
{
	bool success = true;

	for (auto cam : camera_data)
	{
		success = success && ErrorLogger::EXECUTE("Cycle Camera Backward", cam, &Abstract_Data::CycleCaptureBackwards);
	}

	return success;
}

bool MKV_Rendering::CameraManager::AllCamerasSeekTimestamp(uint64_t timestamp)
{
	bool success = true;

	for (auto cam : camera_data)
	{
		success = success && ErrorLogger::EXECUTE("Camera Seek Time", cam, &Abstract_Data::SeekToTime, timestamp);
	}

	return success;
}

open3d::t::geometry::TriangleMesh MKV_Rendering::CameraManager::GetMeshAtTimestamp(VoxelGridData* data, uint64_t timestamp)
{
	open3d::core::Device device(data->device_code);

	open3d::t::geometry::TSDFVoxelGrid voxel_grid(

		{
			{"tsdf", open3d::core::Dtype::Float32},
			{"weight", open3d::core::Dtype::UInt16},
			{"color", open3d::core::Dtype::UInt16}
		},

		data->voxel_size, data->signed_distance_field_truncation,
		16, data->blocks, device
	);

	for (auto cam : camera_data)
	{
		ErrorLogger::EXECUTE("Find Frame At Time " + std::to_string(timestamp), cam, &Abstract_Data::SeekToTime, timestamp);

		ErrorLogger::EXECUTE("Pack Frame into Voxel Grid", cam, &Abstract_Data::PackIntoVoxelGrid, &voxel_grid, data);
	}

	return voxel_grid.ExtractSurfaceMesh(0.0f);
}

std::vector<open3d::geometry::RGBDImage> MKV_Rendering::CameraManager::ExtractImageVectorAtTimestamp(uint64_t timestamp)
{
	std::vector<open3d::geometry::RGBDImage> to_return;

	for (auto cam : camera_data)
	{
		ErrorLogger::EXECUTE("Find Frame At Time " + std::to_string(timestamp), cam, &Abstract_Data::SeekToTime, timestamp);

		to_return.push_back(*ErrorLogger::EXECUTE("Extract RGBD Image Vector", cam, &Abstract_Data::GetFrameRGBD));
	}

	return to_return;
}

void MKV_Rendering::CameraManager::GetTrajectories(open3d::camera::PinholeCameraTrajectory &traj)
{
	for (auto cam : camera_data)
	{
		open3d::camera::PinholeCameraParameters params;

		traj.parameters_.push_back(
			cam->GetParameters()
		);
	}
}

void MKV_Rendering::CameraManager::CreateSSMVFolder(
	VoxelGridData *vgd, std::string destination_folder, uint64_t timestamp, std::string camera_calib_filename, std::string image_list_filename, std::string image_base_name, std::string mesh_name
)
{
	std::ofstream camera_data_file;
	std::ofstream image_data_file;

	std::filesystem::create_directories(destination_folder);

	camera_data_file.open(destination_folder + "/" + camera_calib_filename);
	image_data_file.open(destination_folder + "/" + image_list_filename);

	camera_data_file << camera_data.size() << std::endl;

	for (int i = 0; i < camera_data.size(); ++i)
	{
		auto int_mat = camera_data[i]->GetIntrinsicMat();
		auto ext_mat = camera_data[i]->GetExtrinsicMat();

		for (int j = 0; j < 3; ++j)
		{
			for (int k = 0; k < 3; ++k)
			{
				camera_data_file << int_mat(j, k) << " ";
			}
		}

		for (int j = 0; j < 3; ++j)
		{
			for (int k = 0; k < 3; ++k)
			{
				camera_data_file << ext_mat(j, k) << " ";
			}
		}

		for (int j = 0; j < 3; ++j)
		{
			camera_data_file << ext_mat(j, 3) << " ";
		}

		camera_data[i]->SeekToTime(timestamp);
		auto rgbd_image = camera_data[i]->GetFrameRGBD();

		camera_data_file << rgbd_image->color_.width_ << " ";
		camera_data_file << rgbd_image->color_.height_ << std::endl;



		std::string image_name = GetNumberFixedLength(i, 8) + image_base_name;
		std::string image_address = destination_folder + "/" + image_name;

		open3d::io::WriteImageToPNG(image_address, rgbd_image->color_);

		image_data_file << image_address << std::endl;
	}

	auto mesh = GetMeshAtTimestamp(vgd, timestamp).ToLegacyTriangleMesh();

	open3d::io::WriteTriangleMesh(destination_folder + "/" + mesh_name, mesh);

	image_data_file.close();
	camera_data_file.close();
}

uint64_t MKV_Rendering::CameraManager::GetHighestTimestamp()
{
	uint64_t to_return = 0;
	
	for (auto cam : camera_data)
	{
		auto timestamp = cam->GetTimestampCached();

		if (to_return < timestamp)
		{
			to_return = timestamp;
		}
	}

	return to_return;
}

void MKV_Rendering::CameraManager::MakeAnErrorOnPurpose(bool cause_abort)
{
	CauseError(cause_abort);
}
