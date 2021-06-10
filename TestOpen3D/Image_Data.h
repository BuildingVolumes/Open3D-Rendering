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
	class Image_Data : public Abstract_Data
	{
		static Image_Data* main_camera_data;

		std::vector<std::string> color_files;
		std::map<uint64_t, std::string> color_timestamps;
		std::vector<std::string> depth_files;
		std::map<uint64_t, std::string> depth_timestamps;

		std::string color_folder;
		std::string depth_folder;
		std::string calibration_file;
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
		Image_Data(std::string root_folder, std::string color_folder, std::string depth_folder, std::string intrinsics, std::string extrinsics, std::string FPS);
		~Image_Data();

		uint64_t GetCaptureTimestamp();
		bool CycleCaptureForwards();
		bool CycleCaptureBackwards();
		bool SeekToTime(uint64_t time);

		std::shared_ptr<open3d::geometry::RGBDImage> GetFrameRGBD();

		open3d::camera::PinholeCameraParameters GetParameters();

		void PackIntoVoxelGrid(open3d::t::geometry::TSDFVoxelGrid* grid, VoxelGridData* data);
	};
}