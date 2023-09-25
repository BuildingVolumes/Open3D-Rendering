#pragma once

#include "Abstract_Data.h"

#include "VoxelGridData.h"

#include <vector>
#include <string>
#include <map>

namespace MKV_Rendering {
	
	/// <summary>
	/// Houses all the cameras, and sends group commands to all of them
	/// </summary>
	class CameraManager {
		/// <summary>
		/// All camera images loaded into this manager
		/// </summary>
		std::vector<Abstract_Data*> camera_data;

		/// <summary>
		/// Allows certain cameras to be excluded from calculations
		/// </summary>
		std::vector<bool> camera_enabled;

		/// <summary>
		/// Are we loaded?
		/// </summary>
		bool loaded = false;

		/// <summary>
		/// Causes an error, use wisely
		/// </summary>
		/// <param name="cause_abort">: Allows easy toggling, a simple parameter change</param>
		void CauseError(bool cause_abort);

		/// <summary>
		/// The .structure file dictates critical information regarding setup
		/// </summary>
		/// <param name="structure_path">: path to the structure file</param>
		/// <param name="data">: data to be read from the file</param>
		void LoadStructure(std::string structure_path, std::map<std::string, std::string> *data);
	public:

		/// <summary>
		/// Default constructor, say hi! :D
		/// </summary>
		CameraManager();

		/// <summary>
		/// Loading images if the structure file is Default
		/// </summary>
		/// <param name="root_folder">: root folder</param>
		/// <param name="structure_file_name">: structure file</param>
		/// <returns>Successfully(?) loaded the files</returns>
		bool LoadTypeStructure(std::string root_folder, std::string structure_file_name);

		/// <summary>
		/// Loading images if the structure file is Livescan
		/// </summary>
		/// <param name="image_root_folder">: folder containing color/depth images</param>
		/// <param name="matte_root_folder">: folder containing matte images</param>
		/// <param name="FPS">: playback speed of camera</param>
		/// <returns>Successfully(?) loaded the files</returns>
		bool LoadTypeLivescan();

		bool IsLoaded() { return loaded; }

		bool AddCameraLivescan(std::string root_folder, std::string intrinsics_file, std::string extrinsics_file,
			std::string color_handle, std::string depth_handle, std::string matte_handle, float FPS);

		//Default destructor, say goodbye :(
		~CameraManager();

		/// <summary>
		/// Destroy/reset all resources allocated on construction
		/// </summary>
		/// <returns>Successfully(?) cleaned up</returns>
		bool Unload();

		/// <summary>
		/// Gets a single mesh from the Open3D voxel grid
		/// </summary>
		/// <param name="data">: data that the voxel grid may need to know</param>
		/// <returns>A mesh</returns>
		open3d::t::geometry::TriangleMesh GetMesh(VoxelGridData* data);

		std::shared_ptr<MeshingVoxelGrid> GetNewVoxelGrid(MeshingVoxelParams params);

		void PackNewVoxelGrid(MeshingVoxelGrid* mvg);

		void PackNewVoxelGridAtTimestamp(MeshingVoxelGrid* mvg, uint64_t timestamp);

		std::shared_ptr<MeshingVoxelGrid> GetNewVoxelGridAtTimestamp(MeshingVoxelParams params, uint64_t timestamp);

		std::shared_ptr<open3d::geometry::PointCloud> GetPointCloud();

		std::shared_ptr<open3d::geometry::PointCloud> GetPointCloudAtTimestamp(uint64_t timestamp);

		/// <summary>
		/// Gets a single mesh from our new voxel grid
		/// </summary>
		/// <param name="params">: grid parameters</param>
		/// <param name="maximum_artifact_size">: max culling size for artifacts</param>
		/// <returns>A pointer to a mesh</returns>
		std::shared_ptr<open3d::geometry::TriangleMesh> GetMeshUsingNewVoxelGrid(MeshingVoxelParams params, int maximum_artifact_size);

		/// <summary>
		/// Gets a single mesh at a specific timestamp from our new voxel grid
		/// </summary>
		/// <param name="params">: grid parameters</param>
		/// <param name="maximum_artifact_size">: max culling size for artifacts</param>
		/// <param name="timestamp">: time in playback</param>
		/// <returns>A pointer to a mesh</returns>
		std::shared_ptr<open3d::geometry::TriangleMesh> GetMeshUsingNewVoxelGridAtTimestamp(MeshingVoxelParams params, int maximum_artifact_size, uint64_t timestamp);

		/// <summary>
		/// Gets an old Open3D voxel grid
		/// </summary>
		/// <param name="data">: data that the voxel grid may need to know</param>
		/// <returns>The voxel grid</returns>
		open3d::geometry::VoxelGrid GetOldVoxelGrid(VoxelGridData* data);

		/// <summary>
		/// Gets a new Open3D voxel grid
		/// </summary>
		/// <param name="data">: data that the voxel grid may need to know</param>
		/// <returns>The voxel grid</returns>
		open3d::t::geometry::TSDFVoxelGrid GetVoxelGrid(VoxelGridData* data);

		/// <summary>
		/// Generates UV's for a mesh created by this object, and an associated texture
		/// </summary>
		/// <param name="mesh">: the mesh to use</param>
		/// <param name="useTheBadTexturingMethod">: there is a good way (small texture) and a bad way (large texture) to do this, both can be done</param>
		/// <returns>The texture produced from this operation</returns>
		std::shared_ptr<open3d::geometry::Image> CreateUVMapAndTexture(open3d::geometry::TriangleMesh* mesh, bool useTheBadTexturingMethod);//, float depth_epsilon = 0.01f);

		/// <summary>
		/// Generates UV's for a mesh created by this object, and an associated texture at a specific timestamp
		/// </summary>
		/// <param name="mesh">: the mesh to use</param>
		/// <param name="timestamp">: the time in the playback</param>
		/// <param name="useTheBadTexturingMethod">: there is a good way (small texture) and a bad way (large texture) to do this, both can be done</param>
		/// <returns></returns>
		std::shared_ptr<open3d::geometry::Image> CreateUVMapAndTextureAtTimestamp(open3d::geometry::TriangleMesh* mesh, uint64_t timestamp, bool useTheBadTexturingMethod);// , float depth_epsilon);

		/// <summary>
		/// Cycle all cameras forward one timestamp
		/// </summary>
		/// <returns>Successfully(?) cycled</returns>
		bool CycleAllCamerasForward();

		/// <summary>
		/// Cycle all cameras backward one timestamp
		/// </summary>
		/// <returns>Successfully(?) cycled</returns>
		bool CycleAllCamerasBackward();

		/// <summary>
		/// Sets all cameras to one, synchronized timestamp
		/// </summary>
		/// <param name="timestamp">: time in playback</param>
		/// <returns>Successfully(?) set</returns>
		bool AllCamerasSeekTimestamp(uint64_t timestamp);

		bool AllCamerasSeekFrame(int frame);

		int GetPlayableFrameCount();

		/// <summary>
		/// Gets a single mesh at a specific timestamp from the Open3D voxel grid
		/// </summary>
		/// <param name="data">: data that the voxel grid may need to know</param>
		/// <param name="timestamp">: time in playback</param>
		/// <returns>The mesh</returns>
		open3d::t::geometry::TriangleMesh GetMeshAtTimestamp(VoxelGridData* data, uint64_t timestamp);

		/// <summary>
		/// Gets a voxel grid at a specific timestamp from the Open3D voxel grid
		/// </summary>
		/// <param name="data">: data that the voxel grid may need to know</param>
		/// <param name="timestamp">: time in playback</param>
		/// <returns>The voxel grid</returns>
		open3d::t::geometry::TSDFVoxelGrid GetVoxelGridAtTimestamp(VoxelGridData* data, uint64_t timestamp);

		/// <summary>
		/// Gets a set of RGBD images across all child cameras at a specific timestamp
		/// </summary>
		/// <param name="timestamp">: time in playback</param>
		/// <returns>The image vector</returns>
		std::vector<open3d::geometry::RGBDImage> ExtractImageVectorAtTimestamp(uint64_t timestamp);

		/// <summary>
		/// Trajectory values from all child cameras
		/// </summary>
		/// <param name="traj">: variable to store trajectories in</param>
		void GetTrajectories(open3d::camera::PinholeCameraTrajectory &traj);

		/// <summary>
		/// Deprecated
		/// </summary>
		/// <param name="vgd"></param>
		/// <param name="destination_folder"></param>
		/// <param name="timestamp"></param>
		/// <param name="camera_calib_filename"></param>
		/// <param name="image_list_filename"></param>
		/// <param name="image_base_name"></param>
		/// <param name="mesh_name"></param>
		void CreateSSMVFolder(VoxelGridData *vgd, 
			std::string destination_folder, uint64_t timestamp, 
			std::string camera_calib_filename = "camera_calibrations.txt", 
			std::string image_list_filename = "image_list.txt", 
			std::string image_base_name = "image.png",
			std::string mesh_name = "SSMV_Mesh.obj");

		/// <summary>
		/// Allows manual enabling/disabling of cameras
		/// </summary>
		/// <param name="index">The index of the camera to enable/disable</param>
		/// <param name="enabled">Enabled state</param>
		void SetCameraEnabled(int index, bool enabled);

		/// <summary>
		/// Gets the highest timestamp across all child cameras, at their current frame
		/// </summary>
		/// <returns>The largest timestamp</returns>
		uint64_t GetHighestTimestamp();

		/// <summary>
		/// Please don't call this :)
		/// </summary>
		/// <param name="cause_abort">: please don't :)</param>
		void MakeAnErrorOnPurpose(bool cause_abort);

		/// <summary>
		/// The height of the images provided by the child cameras, assumed to be the same across all cameras
		/// </summary>
		/// <returns>The height in pixels</returns>
		int GetImageHeight() { return camera_data[0]->GetImageHeight(); }
		
		/// <summary>
		/// The width of the images provided by the child cameras, assumed to be the same across all cameras
		/// </summary>
		/// <returns>The width in pixels</returns>
		int GetImageWidth() { return camera_data[0]->GetImageWidth(); }
	};
}