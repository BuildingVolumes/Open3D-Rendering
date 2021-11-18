#pragma once

#include "open3d/Open3D.h"
#include "Abstract_Data.h"

#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <string>

namespace MKV_Rendering {

	/// <summary>
	/// Old class used for taking RGB and depth data from MKV files
	/// </summary>
	class MKV_Data : public Abstract_Data
	{
		/// <summary>
		/// The master camera, so that all other cameras can copy certain data from it
		/// </summary>
		static MKV_Data* main_camera_data;

		/// <summary>
		/// the mkv filename
		/// </summary>
		std::string mkv_file;

		/// <summary>
		/// the calibration filename
		/// </summary>
		std::string calibration_file;

		/// <summary>
		/// Handle of the playback
		/// </summary>
		k4a_playback_t handle;

		/// <summary>
		/// Handle of the camera's calibration
		/// </summary>
		k4a_calibration_t calibration;

		/// <summary>
		/// Handle of the configuration
		/// </summary>
		k4a_record_configuration_t record_config;

		/// <summary>
		/// Handle of the camera's transform
		/// </summary>
		k4a_transformation_t transform = NULL;

		/// <summary>
		/// Handle of the current capture
		/// </summary>
		k4a_capture_t* capture = nullptr;

		/// <summary>
		/// Raw playback data
		/// </summary>
		std::vector<uint8_t> playback_data;

		/// <summary>
		/// The initial offset of the camera
		/// </summary>
		uint64_t start_offset = 0;

		/// <summary>
		/// Initializes this object
		/// </summary>
		/// <param name="my_folder">: the source folder of the mkv</param>
		/// <param name="mkv_name">: name of the mkv</param>
		/// <param name="calibration_name">: name of the calibration file</param>
		void Initialize(std::string my_folder, std::string mkv_name, std::string calibration_name);

		/// <summary>
		/// Calculates the camera calibration
		/// </summary>
		void Calibrate();

		/// <summary>
		/// Fills the raw playback data vector
		/// </summary>
		void GetPlaybackDataRaw();

		/// <summary>
		/// Loads the intrinsic data from the camera calibration
		/// </summary>
		void GetIntrinsicTensor();
		
		/// <summary>
		/// Loads the extrinsic data from the camera calibration
		/// </summary>
		void GetExtrinsicTensor();

		/// <summary>
		/// Function used to invert the color channels of the MKV for Open3D to use
		/// </summary>
		/// <param name="bgra">: reference to the BGRA source image</param>
		/// <param name="rgb">: reference to the RGB destination image</param>
		void ConvertBGRAToRGB(open3d::geometry::Image& bgra, open3d::geometry::Image& rgb);

		/// <summary>
		/// Reaads the capture for us
		/// </summary>
		/// <returns>A pointer to a single RGBD image</returns>
		std::shared_ptr<open3d::geometry::RGBDImage> DecompressCapture();

	public:
		/// <summary>
		/// Constructor. Say hi! :D
		/// </summary>
		/// <param name="my_folder">: root folder of data</param>
		/// <param name="preferred_mkv_name">: name of mkv file - if provided file is "", will take the first MKV it finds</param>
		/// <param name="preferred_calibration_name">: name of calibration file - if provided file is "", will take the first .log file it finds</param>
		MKV_Data(std::string my_folder, std::string preferred_mkv_name, std::string preferred_calibration_name, int index);
		
		//Destructor. Say goodbye! :(
		~MKV_Data();

		uint64_t GetStartOffset() { return start_offset; }

		//See Abstract_Data for below

		uint64_t GetCaptureTimestamp();
		bool CycleCaptureForwards();
		bool CycleCaptureBackwards();
		bool SeekToTime(uint64_t time);

		std::shared_ptr<open3d::geometry::RGBDImage> GetFrameRGBD();

		open3d::camera::PinholeCameraParameters GetParameters();

		void WriteIntrinsics(std::string filename);

		void PackIntoVoxelGrid(open3d::t::geometry::TSDFVoxelGrid* grid, VoxelGridData* data);

		void PackIntoOldVoxelGrid(open3d::geometry::VoxelGrid* grid);
	};
}

