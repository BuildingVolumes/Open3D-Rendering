#pragma once

#include "open3d/Open3D.h"
#include "VoxelGridData.h"
#include "ErrorLogger.h"

#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <string>
#include <vector>
#include <iostream>
#include "MeshingVoxelGrid.h"

namespace MKV_Rendering {

	/// <summary>
	/// Default class that all other camera data inherits from - defines a single camera
	/// </summary>
	class Abstract_Data
	{
	protected:
		/// <summary>
		/// This camera's intrinsic matrix as a tensor
		/// </summary>
		open3d::core::Tensor intrinsic_t;

		/// <summary>
		/// This camera's extrinsic matrix as a tensor
		/// </summary>
		open3d::core::Tensor extrinsic_t;

		/// <summary>
		/// This camera's extrinsic matrix as a mat4
		/// </summary>
		Eigen::Matrix4d extrinsic_mat;

		/// <summary>
		/// This camera's intrinsic matrix as a mat3
		/// </summary>
		Eigen::Matrix3d intrinsic_mat;

		/// <summary>
		/// Where to find all the images of this camera
		/// </summary>
		std::string folder_name;

		/// <summary>
		/// Current playback time
		/// </summary>
		uint64_t _timestamp = 0;

		/// <summary>
		/// The height of this camera's images
		/// </summary>
		int imageHeight = 0;

		/// <summary>
		/// The width of this camera's images
		/// </summary>
		int imageWidth = 0;

		/// <summary>
		/// The camera's index in the manager
		/// </summary>
		int index = -1;

		int frame_count = 0;

		bool cameraOK = false;
	public:
		/// <summary>
		/// Constructor. Say hi! :D
		/// </summary>
		/// <param name="my_folder">: Where to find the images</param>
		Abstract_Data(std::string my_folder, int index);

		/// <summary>
		/// Parses the time of the current capture
		/// </summary>
		/// <returns>The timestamp</returns>
		virtual uint64_t GetCaptureTimestamp() = 0;

		/// <summary>
		/// Jumps to the next consecutive image
		/// </summary>
		/// <returns>Successfully(?) jumped</returns>
		virtual bool CycleCaptureForwards() = 0;

		/// <summary>
		/// Jumps to the previous consecutive image
		/// </summary>
		/// <returns>Successfully(?) jumped</returns>
		virtual bool CycleCaptureBackwards() = 0;

		/// <summary>
		/// Jumps to the image whose timestamp is the closest to the current time, rounded up
		/// </summary>
		/// <param name="time">: Time to jump to</param>
		/// <returns>Successfully(?) jumped</returns>
		virtual bool SeekToTime(uint64_t time) = 0;

		virtual bool SeekToFrame(int frame) = 0;

		int GetFrameCount() {
			return frame_count;
		}

		/// <summary>
		/// Gets a single RGBD image from the livescan data
		/// </summary>
		/// <returns>Pointer to RGBD image</returns>
		virtual std::shared_ptr<open3d::geometry::RGBDImage> GetFrameRGBD() = 0;

		/// <summary>
		/// Gets the pinhole camera parameters
		/// </summary>
		/// <returns>Pinhole camera parameters</returns>
		virtual open3d::camera::PinholeCameraParameters GetParameters() = 0;

		/// <summary>
		/// Inserts frame data into a target Open3D voxel grid
		/// </summary>
		/// <param name="grid">: the voxel grid</param>
		/// <param name="data">: additional data needed for packing</param>
		virtual void PackIntoVoxelGrid(open3d::t::geometry::TSDFVoxelGrid* grid, VoxelGridData* data) = 0;

		/// <summary>
		/// Inserts frame data into a target Open3D old voxel grid
		/// </summary>
		/// <param name="grid">: the voxel grid</param>
		virtual void PackIntoOldVoxelGrid(open3d::geometry::VoxelGrid* grid) = 0;

		/// <summary>
		/// Inserts frame data into a target voxel grid of our own style
		/// </summary>
		/// <param name="grid">: the voxel grid</param>
		virtual void PackIntoNewVoxelGrid(MeshingVoxelGrid* grid)
		{

		}

		virtual void PackIntoPointCloud(open3d::geometry::PointCloud* cloud)
		{

		}

		open3d::core::Tensor GetIntrinsic();
		open3d::core::Tensor GetExtrinsic();

		Eigen::Matrix4d GetExtrinsicMat() { return extrinsic_mat; }
		Eigen::Matrix3d GetIntrinsicMat() { return intrinsic_mat; }

		uint64_t GetTimestampCached() { return _timestamp; }

		int GetImageHeight() { return imageHeight; }

		int GetImageWidth() { return imageWidth; }

		int GetIndex() { return index; }

		bool IsOK() { return cameraOK; }

		/// <summary>
		/// All-purpose tool to debug Open3D objects to the screen
		/// </summary>
		/// <typeparam name="T">: The type of Open3D object</typeparam>
		/// <param name="object_to_draw">: The object to draw</param>
		template<class T>
		void DrawObject(T& object_to_draw)
		{
			std::vector<std::shared_ptr<const open3d::geometry::Geometry>> to_draw;

			auto object_ptr = std::make_shared<T>(
				object_to_draw);

			to_draw.push_back(object_ptr);

			open3d::visualization::DrawGeometries(to_draw);
		}

	};
}