#include "CameraManager.h"
#include "AdditionalUtilities.h"
#include "ErrorLogger.h"

#include "MKV_Data.h"
#include "Image_Data.h"

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
	for (auto _folder : all_folders)
	{
		camera_data.push_back(new MKV_Data(_folder));
	}

	if (camera_data.size() == 0)
	{
		ErrorLogger::LOG_ERROR(
			"No mkv files present!", true
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
