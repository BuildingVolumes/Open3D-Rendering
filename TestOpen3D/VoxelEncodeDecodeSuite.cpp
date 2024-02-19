#include "VoxelEncodeDecodeSuite.h"
#include "AdditionalUtilities.h"

void VoxelEncodeDecodeSuite::SingleEncodeDecode()
{
	MeshingVoxelParams mvp;

	int voxel_count = 64;

	mvp.center = Eigen::Vector3d(0, 0, 0);
	mvp.points_x = voxel_count + 1;
	mvp.points_y = 2 * voxel_count + 1;
	mvp.points_z = voxel_count + 1;
	mvp.voxel_size = 1.05 / (double)voxel_count;

	std::string save_name = "TEST_DECODE_SEQ" + std::to_string(voxel_count);
	std::string new_obj_name = "TEST_DECODE_VOXEL_" + std::to_string(voxel_count) + ".obj";

	std::shared_ptr<open3d::geometry::TriangleMesh> mesh;

	//vs.LoadImageSequences("_TEST_DATA/july15-spinninghogue_0", "Intrinsics_Calib_", "Extrinsics_", "Color_", "Depth_", ".matte");
	//vs.SetVoxelGridParams(mvp);
	//
	//mesh = vs.GetVoxelGridAtFrame(200)->ExtractMesh();
	//DrawObject(*mesh);
	//
	//std::cout << "Saving Grid..." << std::endl;
	//vs.SaveVolumeStream(save_name, SaveFileFormat::ZIP);

	std::cout << "Loading Grid..." << std::endl;
	vs.LoadVolumeStream(save_name + ".vgzf");

	//auto grid = vs.LoadFrameFromStream(200);

	std::cout << "Creating grid..." << std::endl;
	auto grid = vs.LoadFrameFromStream(200);

	std::cout << "Grid total: " << grid->Sum() << std::endl;
	
	std::cout << "Extracting mesh..." << std::endl;
	mesh = grid->ExtractMesh();

	std::cout << "verts: " << mesh->vertices_.size() << ", tris: " << mesh->triangles_.size() << std::endl;

	DrawObject(*mesh);

	//open3d::io::WriteTriangleMeshToOBJ(new_obj_name, *mesh, true, false, true, false, true, false);

	vs.CloseLoadedVolumeStream();
}

void VoxelEncodeDecodeSuite::run(int argc, char** argv)
{
	SingleEncodeDecode();
}
