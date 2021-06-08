#pragma once

#include "Abstract_Data.h"

#include "VoxelGridData.h"

#include <vector>
#include <string>
#include <map>

namespace MKV_Rendering {
	class CameraManager {
		std::vector<Abstract_Data*> camera_data;

		void CauseError(bool cause_abort);

		void LoadStructure(std::string structure_path, std::map<std::string, std::string> *data);
	public:
		CameraManager(std::string root_folder, std::string structure_file_name);

		~CameraManager();

		open3d::t::geometry::TriangleMesh GetMeshAtTimestamp(VoxelGridData* data, uint64_t timestamp);

		std::vector<open3d::geometry::RGBDImage> ExtractImageVectorAtTimestamp(uint64_t timestamp);

		void GetTrajectories(open3d::camera::PinholeCameraTrajectory &traj);

		uint64_t GetHighestTimestamp();

		void MakeAnErrorOnPurpose(bool cause_abort);
	};
}