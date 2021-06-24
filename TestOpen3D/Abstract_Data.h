#pragma once

#include "open3d/Open3D.h"
#include "VoxelGridData.h"
#include "ErrorLogger.h"

#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <string>
#include <vector>
#include <iostream>

namespace MKV_Rendering {
	class Abstract_Data
	{
	protected:
		open3d::core::Tensor intrinsic_t;
		open3d::core::Tensor extrinsic_t;

		Eigen::Matrix4d extrinsic_mat;
		Eigen::Matrix3d intrinsic_mat;

		std::string folder_name;

		uint64_t _timestamp = 0;

		int imageHeight = 0;
		int imageWidth = 0;
	public:
		Abstract_Data(std::string my_folder);

		virtual uint64_t GetCaptureTimestamp() = 0;
		virtual bool CycleCaptureForwards() = 0;
		virtual bool CycleCaptureBackwards() = 0;
		virtual bool SeekToTime(uint64_t time) = 0;

		virtual std::shared_ptr<open3d::geometry::RGBDImage> GetFrameRGBD() = 0;

		virtual open3d::camera::PinholeCameraParameters GetParameters() = 0;

		virtual void PackIntoVoxelGrid(open3d::t::geometry::TSDFVoxelGrid* grid, VoxelGridData* data) = 0;

		virtual void PackIntoOldVoxelGrid(open3d::geometry::VoxelGrid* grid) = 0;

		open3d::core::Tensor GetIntrinsic();
		open3d::core::Tensor GetExtrinsic();

		Eigen::Matrix4d GetExtrinsicMat() { return extrinsic_mat; }
		Eigen::Matrix3d GetIntrinsicMat() { return intrinsic_mat; }

		uint64_t GetTimestampCached() { return _timestamp; }

		int GetImageHeight() { return imageHeight; }

		int GetImageWidth() { return imageWidth; }
	};
}