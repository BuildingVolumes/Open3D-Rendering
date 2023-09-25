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

	/// <summary>
	/// Data split up into multiple folders containing images
	/// </summary>
	class Image_Data : public Abstract_Data
	{
		/// <summary>
		/// The master camera, so that all other cameras can copy certain data from it
		/// </summary>
		static Image_Data* main_camera_data;

		/// <summary>
		/// Vector of color images in order
		/// </summary>
		std::vector<std::string> color_files;

		/// <summary>
		/// Map of color image names to their timestamps
		/// </summary>
		std::map<uint64_t, std::string> color_timestamps;

		/// <summary>
		/// Vector of depth images in order
		/// </summary>
		std::vector<std::string> depth_files;

		/// <summary>
		/// Map of depth image names to their timestamps
		/// </summary>
		std::map<uint64_t, std::string> depth_timestamps;

		std::string color_folder;
		std::string depth_folder;
		std::string calibration_file;
		std::string intrinsics_file;

		/// <summary>
		/// The playback handle
		/// </summary>
		k4a_playback_t handle;

		/// <summary>
		/// The camera calibration
		/// </summary>
		k4a_calibration_t calibration;

		/// <summary>
		/// the camera's transformation
		/// </summary>
		k4a_transformation_t transform = NULL;

		/// <summary>
		/// Playback speed
		/// </summary>
		double FPS;

		/// <summary>
		/// The current map element being used
		/// </summary>
		size_t current_frame = 0;

		/// <summary>
		/// Updates the timestamp
		/// </summary>
		void UpdateTimestamp();

		/// <summary>
		/// Loads all color/depth image names
		/// </summary>
		void LoadImages();

		/// <summary>
		/// Loads the data from the intrinsics file
		/// </summary>
		void GetIntrinsicTensor();

		/// <summary>
		/// Loads the data from the extrinsics file
		/// </summary>
		void GetExtrinsicTensor();

		open3d::geometry::Image TransformDepth(open3d::geometry::Image* old_depth, open3d::geometry::Image* color);
	public:
		/// <summary>
		/// Constructor - say hi! :D
		/// </summary>
		/// <param name="root_folder">: the folder containing all the data</param>
		/// <param name="color_folder">: the subfolder for color</param>
		/// <param name="depth_folder">: the subfolder for depth</param>
		/// <param name="intrinsics">: the intrinsics file name (in .json)</param>
		/// <param name="extrinsics">: the extrinsics file name (in .log)</param>
		/// <param name="FPS">: playback speed</param>
		Image_Data(std::string root_folder, std::string color_folder, std::string depth_folder, std::string intrinsics, std::string extrinsics, std::string FPS, int index);
		
		//Destructor - say goodbye! :(
		~Image_Data();

		//See Abstract_Data for below

		uint64_t GetCaptureTimestamp();
		bool CycleCaptureForwards();
		bool CycleCaptureBackwards();
		bool SeekToTime(uint64_t time);
		bool SeekToFrame(int frame);

		std::shared_ptr<open3d::geometry::RGBDImage> GetFrameRGBD();

		open3d::camera::PinholeCameraParameters GetParameters();

		void PackIntoVoxelGrid(open3d::t::geometry::TSDFVoxelGrid* grid, VoxelGridData* data);

		void PackIntoOldVoxelGrid(open3d::geometry::VoxelGrid* grid);
	};
}