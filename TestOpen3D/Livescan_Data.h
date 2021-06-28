#pragma once

#include "open3d/Open3D.h"
#include "VoxelGridData.h"
#include "Abstract_Data.h"

#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <string>
#include <vector>
#include <map>

namespace MKV_Rendering {
	class Livescan_Data : public Abstract_Data
	{
		std::vector<std::string> color_files;
		std::vector<std::string> depth_files;

		std::vector<std::string> extrinsic_individual;

		std::string intrinsics_file;

		k4a_playback_t handle;
		k4a_calibration_t calibration;
		k4a_transformation_t transform = NULL;

		double FPS;
		size_t current_frame = 0;

		void UpdateTimestamp();
		void LoadImages();

		void GetIntrinsicTensor();
		void GetExtrinsicTensor();

		open3d::geometry::Image TransformDepth(open3d::geometry::Image* old_depth, open3d::geometry::Image* color);
	public:
		Livescan_Data(std::string data_folder, std::vector<std::string> &extrinsics, double FPS);
		~Livescan_Data();

		uint64_t GetCaptureTimestamp();
		bool CycleCaptureForwards();
		bool CycleCaptureBackwards();
		bool SeekToTime(uint64_t time);

		std::shared_ptr<open3d::geometry::RGBDImage> GetFrameRGBD();

		open3d::camera::PinholeCameraParameters GetParameters();

		void PackIntoVoxelGrid(open3d::t::geometry::TSDFVoxelGrid* grid, VoxelGridData* data);

		void PackIntoOldVoxelGrid(open3d::geometry::VoxelGrid* grid);
	};
}