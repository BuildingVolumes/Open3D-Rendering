#pragma once

#include <string>
#include "CameraManager.h"
#include "MeshingVoxelGrid.h"

enum SaveFileFormat
{
	FOURIER,
	ZIP
};

class VolumeSequence
{
	MKV_Rendering::CameraManager cm;
	MeshingVoxelParams mvp;

	Eigen::Vector3i trim;

	//std::vector<std::shared_ptr<open3d::visualization::me

	const char* file_ending_fourier = ".vgf";
	const char* file_ending_zip = ".vgz";
	const char* file_ending_flat = "f";
	const char* file_ending_Iframe = "i";

	double threshold = 0;
	double culling_value = 0;

public:

	VolumeSequence();

	~VolumeSequence();

	bool SetVoxelGridParams(MeshingVoxelParams mvp);

	void SetTrim(Eigen::Vector3i new_trim) { trim = new_trim; }
	void SetTrim(int x, int y, int z) { trim = Eigen::Vector3i(x, y, z); }

	Eigen::Vector3i GetTrim() { return trim; }

	void SetIFrameThreshold(double new_threshold) { threshold = new_threshold; }

	void SetFrameCulling(double new_culling_value) { culling_value = new_culling_value; }

	int LoadImageSequences(std::string root_folder, std::string intrinsics_handle, std::string extrinsics_handle, std::string color_handle, std::string depth_handle, std::string matte_handle = "", float FPS = 5.0);

	int SaveVolumeStream(std::string filename_without_extension, SaveFileFormat format, int start_frame = 0, int end_frame = MAXINT32);

	int SaveAllFramesAsMeshes(std::string root_folder, std::string main_file_name, std::string mesh_filename_without_extension);

	void SaveFrameAsMesh(std::string filename, int frame_number);

	//int SaveVolumeStreamFourier(std::string filename, int start_frame = 0, int end_frame = MAXINT32);
	//
	//int SaveVolumeStreamZipAlgorithm(std::string filename, int start_frame = 0, int end_frame = MAXINT32);

	int LoadVolumeStreamAndExtractMeshes(std::string filename);

	std::shared_ptr<MeshingVoxelGrid> GetVoxelGridAtFrame(int frame);

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