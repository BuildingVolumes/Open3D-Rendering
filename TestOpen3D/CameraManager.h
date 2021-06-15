#pragma once

#include "Abstract_Data.h"

#include "VoxelGridData.h"

#include <vector>
#include <string>
#include <map>

namespace MKV_Rendering {
	class CameraManager {
		std::vector<Abstract_Data*> camera_data;

		void CauseError(bool cause_abort);

		void LoadStructure(std::string structure_path, std::map<std::string, std::string> *data);
	public:
		CameraManager(std::string root_folder, std::string structure_file_name);

		~CameraManager();

		open3d::t::geometry::TriangleMesh GetMesh(VoxelGridData* data);

		std::shared_ptr<open3d::geometry::Image> CreateUVMapAndTexture(open3d::geometry::TriangleMesh* mesh);

		std::shared_ptr<open3d::geometry::Image> CreateUVMapAndTextureAtTimestamp(open3d::geometry::TriangleMesh *mesh, uint64_t timestamp);

		bool CycleAllCamerasForward();

		bool CycleAllCamerasBackward();

		bool AllCamerasSeekTimestamp(uint64_t timestamp);

		open3d::t::geometry::TriangleMesh GetMeshAtTimestamp(VoxelGridData* data, uint64_t timestamp);

		std::vector<open3d::geometry::RGBDImage> ExtractImageVectorAtTimestamp(uint64_t timestamp);

		void GetTrajectories(open3d::camera::PinholeCameraTrajectory &traj);

		void CreateSSMVFolder(VoxelGridData *vgd, 
			std::string destination_folder, uint64_t timestamp, 
			std::string camera_calib_filename = "camera_calibrations.txt", 
			std::string image_list_filename = "image_list.txt", 
			std::string image_base_name = "image.png",
			std::string mesh_name = "SSMV_Mesh.obj");

		uint64_t GetHighestTimestamp();

		void MakeAnErrorOnPurpose(bool cause_abort);
	};
}