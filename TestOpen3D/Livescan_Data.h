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
	/// Data that comes from livescan cameras
	/// </summary>
	class Livescan_Data : public Abstract_Data
	{
		/// <summary>
		/// map of timestamp to color image directory/name
		/// </summary>
		std::map<int, std::string> color_files;

		/// <summary>
		/// map of timestamp to depth image directory/name
		/// </summary>
		std::map<int, std::string> depth_files;

		/// <summary>
		/// map of timestamp to matte directory/name
		/// </summary>
		std::map<int, std::string> matte_files;

		/// <summary>
		/// The extrinsic properties of this camera (rotation + position)
		/// </summary>
		std::vector<std::string> extrinsic_individual;

		/// <summary>
		/// Where to find the mattes - will be deprecated soon, assumed same directory
		/// </summary>
		std::string matte_folder_name;

		/// <summary>
		/// Where to find the intrinsics
		/// </summary>
		std::string intrinsics_file;

		/// <summary>
		/// Handle to livescan playback
		/// </summary>
		k4a_playback_t handle;

		/// <summary>
		/// The camera calibration handle
		/// </summary>
		k4a_calibration_t calibration;

		/// <summary>
		/// The camera transform handle
		/// </summary>
		k4a_transformation_t transform = NULL;

		/// <summary>
		/// Playback speed
		/// </summary>
		double FPS;

		/// <summary>
		/// Current frame
		/// </summary>
		size_t current_frame = 0;

		/// <summary>
		/// Caches the current playback time
		/// </summary>
		void UpdateTimestamp();
		void LoadImages();

		void GetIntrinsicTensor();
		void GetExtrinsicTensor();

		open3d::geometry::Image TransformDepth(open3d::geometry::Image* old_depth, open3d::geometry::Image* color);
	public:
		/// <summary>
		/// Constructor. Say hi! :D
		/// </summary>
		/// <param name="data_folder">: where to find the rgb and depth images</param>
		/// <param name="matte_folder">: where to find the matte images, will be deprecated in the future</param>
		/// <param name="extrinsics">: camera extrinsics</param>
		/// <param name="camera_ID">: camera's ID</param>
		/// <param name="FPS">: playback speed</param>
		Livescan_Data(std::string data_folder, std::string matte_folder, std::vector<std::string> &extrinsics, int index, double FPS);

		//Destructor. Say goodbye! :(
		~Livescan_Data();

		//See Abstract_Data for below

		uint64_t GetCaptureTimestamp();
		bool CycleCaptureForwards();
		bool CycleCaptureBackwards();
		bool SeekToTime(uint64_t time);

		std::shared_ptr<open3d::geometry::RGBDImage> GetFrameRGBD();

		open3d::camera::PinholeCameraParameters GetParameters();

		void PackIntoVoxelGrid(open3d::t::geometry::TSDFVoxelGrid* grid, VoxelGridData* data);

		void PackIntoOldVoxelGrid(open3d::geometry::VoxelGrid* grid);

		void PackIntoNewVoxelGrid(MeshingVoxelGrid* grid);
	};
}