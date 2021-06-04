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

		std::vector<uint8_t> playback_data;

		void Initialize(std::string my_folder, std::string mkv_name, std::string calibration_name);
		void Calibrate();
		void GetPlaybackDataRaw();
		void GetIntrinsicTensor();
		void GetExtrinsicTensor();

		void ConvertBGRAToRGB(open3d::geometry::Image& bgra, open3d::geometry::Image& rgb);
		std::shared_ptr<open3d::geometry::RGBDImage> DecompressCapture();

	public:
		MKV_Data(std::string my_folder, std::string preferred_mkv_name, std::string preferred_calibration_name);
		~MKV_Data();

		uint64_t GetCaptureTimestamp();
		bool CycleCaptureForwards();
		bool CycleCaptureBackwards();
		bool SeekToTime(uint64_t time);

		std::shared_ptr<open3d::geometry::RGBDImage> GetFrameRGBD();
		void WriteIntrinsics(std::string filename);

		void PackIntoVoxelGrid(open3d::t::geometry::TSDFVoxelGrid* grid, VoxelGridData* data);
	};
}

