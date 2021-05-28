#pragma once

#include "open3d/Open3D.h"

#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <string>

namespace MKV_Rendering {
	class MKV_Data
	{
		static MKV_Data* main_camera_data;

		std::string mkv_file;
		std::string calibration_file;

		open3d::core::Tensor intrinsic_t;
		open3d::core::Tensor extrinsic_t;

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
		MKV_Data(std::string mkv_file, std::string calibration_file);
		~MKV_Data();

		uint64_t GetCaptureTimestamp();
		void CycleCaptureForwards();
		void CycleCaptureBackwards();
		void SeekToTime(uint64_t time);

		open3d::core::Tensor GetIntrinsic();
		open3d::core::Tensor GetExtrinsic();

		std::shared_ptr<open3d::geometry::RGBDImage> GetFrameRGBD();
		uint64_t GetTimestampCached() { return _timestamp; }
	};
}