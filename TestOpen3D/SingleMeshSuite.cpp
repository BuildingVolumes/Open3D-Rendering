#include "SingleMeshSuite.h"

void SingleMeshSuite::run(int argc, char** argv)
{
	MeshingVoxelParams mvp;

	int voxel_count = 100;

	mvp.center = Eigen::Vector3d(0, 0, 0);
	mvp.points_x = voxel_count + 1;
	mvp.points_y = 2 * voxel_count + 1;
	mvp.points_z = voxel_count + 1;
	mvp.voxel_size = 1.05 / (double)voxel_count;

	vs.LoadImageSequences("_TEST_DATA/july15-spinninghogue_0", "Intrinsics_Calib_", "Extrinsics_", "Color_", "Depth_", ".matte");

	vs.SaveFrameAsMesh("TestMesh_12_13_2023", 200);
}
