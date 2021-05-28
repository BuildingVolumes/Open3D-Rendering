#pragma once

#include "MKV_Data.h"
#include "VoxelGridData.h"

#include <vector>
#include <string>

namespace MKV_Rendering {
	class CameraManager {
		std::vector<MKV_Data*> mkv_data;

		void CauseError(bool cause_abort);
	public:
		CameraManager(std::string root_folder);

		~CameraManager();

		open3d::t::geometry::TriangleMesh GetMeshAtTimestamp(VoxelGridData* data, uint64_t timestamp);

		uint64_t GetHighestTimestamp();

		void MakeAnErrorOnPurpose(bool cause_abort);
	};
}