#include "PointCloudSuite.h"

#include "CameraManager.h"

void PointCloudSuite::run(int argc, char** argv)
{
	double capture_width = 1.2;
	int base_width = 201;
	int base_height = 401;
	MeshingVoxelParams params(capture_width / base_width, base_width, base_height, base_width, Eigen::Vector3d(0, 0, 0));

	//int edge_length = 100;
	//MeshingVoxelParams params(1.0 / (double)(edge_length - 1), edge_length, edge_length, edge_length, Eigen::Vector3d(0, 0, 0));

	auto p_tot = (params.points_x) * (params.points_y) * (params.points_z);

	MKV_Rendering::CameraManager cm;

	cm.AddCameraLivescan("_TEST_DATA/_TestingNewExtrinsics/client_0", "_TEST_DATA/_TestingNewExtrinsics/client_0/Intrinsics_Calib_0.json", "_TEST_DATA/_TestingNewExtrinsics/client_0/Extrinsics_0.log",
		"Color_", "Depth_", "Matte_", 5.0);
	cm.AddCameraLivescan("_TEST_DATA/_TestingNewExtrinsics/client_1", "_TEST_DATA/_TestingNewExtrinsics/client_1/Intrinsics_Calib_1.json", "_TEST_DATA/_TestingNewExtrinsics/client_1/Extrinsics_1.log",
		"Color_", "Depth_", "Matte_", 5.0);
	cm.AddCameraLivescan("_TEST_DATA/_TestingNewExtrinsics/client_2", "_TEST_DATA/_TestingNewExtrinsics/client_2/Intrinsics_Calib_2.json", "_TEST_DATA/_TestingNewExtrinsics/client_2/Extrinsics_2.log",
		"Color_", "Depth_", "Matte_", 5.0);
	cm.AddCameraLivescan("_TEST_DATA/_TestingNewExtrinsics/client_3", "_TEST_DATA/_TestingNewExtrinsics/client_3/Intrinsics_Calib_3.json", "_TEST_DATA/_TestingNewExtrinsics/client_3/Extrinsics_3.log",
		"Color_", "Depth_", "Matte_", 5.0);
	cm.AddCameraLivescan("_TEST_DATA/_TestingNewExtrinsics/client_4", "_TEST_DATA/_TestingNewExtrinsics/client_4/Intrinsics_Calib_4.json", "_TEST_DATA/_TestingNewExtrinsics/client_4/Extrinsics_4.log",
		"Color_", "Depth_", "Matte_", 5.0);
	cm.AddCameraLivescan("_TEST_DATA/_TestingNewExtrinsics/client_5", "_TEST_DATA/_TestingNewExtrinsics/client_5/Intrinsics_Calib_5.json", "_TEST_DATA/_TestingNewExtrinsics/client_5/Extrinsics_5.log",
		"Color_", "Depth_", "Matte_", 5.0);

	MKV_Rendering::VoxelGridData vgd;
	vgd.voxel_size = 9.f / 512.f;

	uint64_t timestamp = 21800000;

	auto cloud = cm.GetPointCloudAtTimestamp(timestamp);

	DrawObject(*cloud);
}
