#pragma once

#include "TestSuite.h"
#include "CameraManager.h"
#include <open3d/Open3D.h>
#include <vector>
#include <string>

class TestZIP : public TestSuite
{
	int camera_count = 6;

	std::string the_date = "06/05/2023";

	std::string path = "_TEST_DATA/_TestingNewExtrinsics/";
	std::vector<std::string> rgbds;
	std::vector<std::string> intrinsics;
	std::vector<std::string> extrinsics;

public:

	void SetPaths()
	{
		for (int i = 0; i < camera_count; ++i)
		{
			std::string i_str = std::to_string(i);

			rgbds.push_back("client_" + i_str);
			intrinsics.push_back(rgbds[i] + "/Intrinsics_Calib_" + i_str + ".json");
			extrinsics.push_back(rgbds[i] + "/Extrinsics_" + i_str + ".log");
		}
	}

	std::shared_ptr<MKV_Rendering::CameraManager> LoadCameras(std::string main_path, std::vector<std::string> subpaths_rgbd, std::vector<std::string> subpaths_intrinsics, std::vector<std::string> subpaths_extrinsics);

	void run(int argc, char** argv);

	void WriteOBJ(std::string filename, open3d::geometry::TriangleMesh* mesh);

	void ZIP_Forward(std::string ZIP_Name);

	void ZIP_Backward(std::string ZIP_Name);
};