#pragma once

#include "open3d/Open3D.h"
#include "Abstract_Data.h"

#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <string>

namespace MKV_Rendering {
	class MKV_Data : public Abstract_Data
	{
		static MKV_Data* main_camera_data;

		std::string mkv_file;
		std::string calibration_file;

		k4a_playback_t handle;
		k4a_calibration_t calibration;
		k4a_record_configuration_t record_config;
		k4a_transformation_t transform = NULL;
		k4a_capture_t* capture = nullptr;

		uint64_t _timestamp = 0;

		void Calibrate();
		void GetPlaybackDataRaw();
		void GetIntrinsicTensor();
		void GetExtrinsicTensor();

		void ConvertBGRAToRGB(open3d::geometry::Image& bgra, open3d::geometry::Image& rgb);
		std::shared_ptr<open3d::geometry::RGBDImage> DecompressCapture();

	public:
		MKV_Data(std::string my_folder);
		~MKV_Data();

		uint64_t GetCaptureTimestamp();
		void CycleCaptureForwards();
		void CycleCaptureBackwards();
		void SeekToTime(uint64_t time);

		std::shared_ptr<open3d::geometry::RGBDImage> GetFrameRGBD();

		void PackIntoVoxelGrid(open3d::t::geometry::TSDFVoxelGrid* grid, VoxelGridData* data);
	};
}