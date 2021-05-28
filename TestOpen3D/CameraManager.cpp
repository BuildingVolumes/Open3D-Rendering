#include "CameraManager.h"
#include "AdditionalUtilities.h"
#include "ErrorLogger.h"

using namespace MKV_Rendering;

void MKV_Rendering::CameraManager::CauseError(bool cause_abort)
{
	bool bad_thing_happened = true;

	if (bad_thing_happened)
	{
		ErrorLogger::LOG_ERROR("DONT WORRY! This error is suppsoed to happen here!", cause_abort);
	}
}

CameraManager::CameraManager(std::string root_folder)
{
	std::vector<std::string> all_folders = GetDirectories(root_folder);
	//open3d::utility::filesystem::ListFilesInDirectory(root_folder, all_folders);

	std::vector<std::string> mkv_files;
	std::vector<std::string> calibration_log_files;
	
	for (auto _folder : all_folders)
	{
		std::vector<std::string> files;
		open3d::utility::filesystem::ListFilesInDirectory(_folder, files);

		for (auto _file : files)
		{
			std::vector<std::string> filename_and_extension;

			SplitString(_file, filename_and_extension, '.');

			if (filename_and_extension.back() == "mkv")
			{
				mkv_files.push_back(_file);
			}
			else if (filename_and_extension.back() == "log")
			{
				calibration_log_files.push_back(_file);
			}
		}
	}

	if (mkv_files.size() != calibration_log_files.size())
	{
		ErrorLogger::LOG_ERROR(
			"Different amouns of mkv files (" +
			std::to_string(mkv_files.size()) +
			") and calibration files (" + 
			std::to_string(calibration_log_files.size()) + 
			")", true
		);
	}
	else if (mkv_files.size() == 0)
	{
		ErrorLogger::LOG_ERROR(
			"No mkv files present!", true
		);
	}

	for (int i = 0; i < mkv_files.size(); ++i)
	{
		mkv_data.push_back(new MKV_Data(mkv_files[i], calibration_log_files[i]));
	}
}

CameraManager::~CameraManager()
{
	while (!mkv_data.empty())
	{
		if (mkv_data.back() != nullptr)
		{
			delete mkv_data.back();
		}

		mkv_data.pop_back();
	}
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

	for (auto mkv : mkv_data)
	{
		ErrorLogger::EXECUTE("Find MKV Frame At Time " + std::to_string(timestamp), mkv, &MKV_Data::SeekToTime, timestamp);

		auto rgbd = mkv->GetFrameRGBD();

		auto color = open3d::t::geometry::Image::FromLegacyImage(rgbd->color_);
		auto depth = open3d::t::geometry::Image::FromLegacyImage(rgbd->depth_);

		color.To(device);
		depth.To(device);

		voxel_grid.Integrate(depth, color, 
			mkv->GetIntrinsic(), mkv->GetExtrinsic(), 
			data->depth_scale, data->depth_max);
	}

	return voxel_grid.ExtractSurfaceMesh(0.0f);
}

uint64_t MKV_Rendering::CameraManager::GetHighestTimestamp()
{
	uint64_t to_return = 0;
	
	for (auto mkv : mkv_data)
	{
		auto timestamp = mkv->GetTimestampCached();

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
