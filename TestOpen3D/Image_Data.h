#pragma once

#include "open3d/Open3D.h"
#include "VoxelGridData.h"
#include "Abstract_Data.h"

#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <string>
#include <vector>

namespace MKV_Rendering {
	class Image_Data : public Abstract_Data
	{
		static Image_Data* main_camera_data;

		std::vector<std::string> color_files;
		std::vector<std::string> depth_files;

		std::string root_folder;
		std::string color_folder;
		std::string depth_folder;
		std::string calibration_file;
		std::string intrinsics_file;

		k4a_playback_t handle;
		k4a_calibration_t calibration;

		double FPS;
		size_t currentFrame = 0;
	public:
		Image_Data(std::string root_folder, std::string color_folder, std::string depth_folder, double FPS);
		~Image_Data();

		uint64_t GetCaptureTimestamp();
		void CycleCaptureForwards();
		void CycleCaptureBackwards();
		void SeekToTime(uint64_t time);

		void PackIntoVoxelGrid(open3d::t::geometry::TSDFVoxelGrid* grid, VoxelGridData* data);
	};
}