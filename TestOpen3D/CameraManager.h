#pragma once

#include "Abstract_Data.h"

#include "VoxelGridData.h"

#include <vector>
#include <string>
#include <map>

namespace MKV_Rendering {
	class CameraManager {
		std::vector<Abstract_Data*> camera_data;

		bool loaded = false;

		void CauseError(bool cause_abort);

		void LoadStructure(std::string structure_path, std::map<std::string, std::string> *data);
	public:
		CameraManager();

		bool LoadTypeStructure(std::string root_folder, std::string structure_file_name);

		bool LoadTypeLivescan(std::string image_root_folder, std::string matte_root_folder, float FPS);

		~CameraManager();

		bool Unload();

		open3d::t::geometry::TriangleMesh GetMesh(VoxelGridData* data);

		std::shared_ptr<open3d::geometry::TriangleMesh> GetMeshUsingNewVoxelGrid();

		std::shared_ptr<open3d::geometry::TriangleMesh> GetMeshUsingNewVoxelGridAtTimestamp(uint64_t timestamp);

		open3d::geometry::VoxelGrid GetOldVoxelGrid(VoxelGridData* data);

		open3d::t::geometry::TSDFVoxelGrid GetVoxelGrid(VoxelGridData* data);

		std::shared_ptr<open3d::geometry::Image> CreateUVMapAndTexture(open3d::geometry::TriangleMesh* mesh, bool useTheBadTexturingMethod);//, float depth_epsilon = 0.01f);

		std::shared_ptr<open3d::geometry::Image> CreateUVMapAndTextureAtTimestamp(open3d::geometry::TriangleMesh* mesh, uint64_t timestamp, bool useTheBadTexturingMethod);// , float depth_epsilon);

		bool CycleAllCamerasForward();

		bool CycleAllCamerasBackward();

		bool AllCamerasSeekTimestamp(uint64_t timestamp);

		open3d::t::geometry::TriangleMesh GetMeshAtTimestamp(VoxelGridData* data, uint64_t timestamp);

		open3d::t::geometry::TSDFVoxelGrid GetVoxelGridAtTimestamp(VoxelGridData* data, uint64_t timestamp);

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

		int GetImageHeight() { return camera_data[0]->GetImageHeight(); }
		
		int GetImageWidth() { return camera_data[0]->GetImageWidth(); }
	};
}