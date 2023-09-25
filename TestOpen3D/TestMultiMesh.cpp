#include "TestMultiMesh.h"

void TestMultiMesh::run(int argc, char** argv)
{
	SaveGridFourier();

	//LoadGridFourier();

	//SaveMultipleGridsFourier();

	//SaveGridZip();

	//SaveGridZipIframes();
}

void TestMultiMesh::SaveGridFourier()
{
	MeshingVoxelParams mvp;

	int voxel_count = 100;

	mvp.center = Eigen::Vector3d(0, 0, 0);
	mvp.points_x = voxel_count + 1;
	mvp.points_y = 2 * voxel_count + 1;
	mvp.points_z = voxel_count + 1;
	mvp.voxel_size = 1.05 / (double)voxel_count;

	int trim = 25;

	std::string save_name = "FOURIER_" + std::to_string(voxel_count) + "_TRIM_" + std::to_string(trim);

	std::cout << "Loading images..." << std::endl;
	//vs.LoadImageSequences("_TEST_DATA/_TestingNewExtrinsics", "Intrinsics_Calib_", "Extrinsics_", "Color_", "Depth_", "Matte_");
	vs.LoadImageSequences("_TEST_DATA/july15-spinninghogue_0", "Intrinsics_Calib_", "Extrinsics_", "Color_", "Depth_", ".matte");

	vs.SetVoxelGridParams(mvp);
	vs.SetTrim(trim + 1, 2 * trim + 1, trim + 1);

	std::cout << "Saving grid..." << std::endl;
	vs.SaveVolumeStream(save_name, SaveFileFormat::FOURIER);
	//vs.SaveVolumeStreamFourier("TestingCompleteFourier.vgff");
	std::cout << "Saved!" << std::endl;
}

void TestMultiMesh::SaveGridZip()
{
	MeshingVoxelParams mvp;

	int voxel_count = 100;

	mvp.center = Eigen::Vector3d(0, 0, 0);
	mvp.points_x = voxel_count + 1;
	mvp.points_y = 2 * voxel_count + 1;
	mvp.points_z = voxel_count + 1;
	mvp.voxel_size = 1.05 / (double)voxel_count;

	std::string save_name = "ZIPALG_" + std::to_string(voxel_count);

	std::cout << "Loading images..." << std::endl;
	//vs.LoadImageSequences("_TEST_DATA/_TestingNewExtrinsics", "Intrinsics_Calib_", "Extrinsics_", "Color_", "Depth_", "Matte_");
	vs.LoadImageSequences("_TEST_DATA/july15-spinninghogue_0", "Intrinsics_Calib_", "Extrinsics_", "Color_", "Depth_", ".matte");

	vs.SetVoxelGridParams(mvp);

	std::cout << "Saving grid..." << std::endl;
	vs.SaveVolumeStream(save_name, SaveFileFormat::ZIP);
	//vs.SaveVolumeStreamZipAlgorithm(save_name);
	std::cout << "Saved!" << std::endl;
}

void TestMultiMesh::LoadGridFourier()
{
	std::cout << "Loading grid..." << std::endl;
	vs.LoadVolumeStreamAndExtractMeshes("TestingCompleteFourier.vgff");
	std::cout << "Loaded!" << std::endl;
}

void TestMultiMesh::SaveMultipleGridsFourier()
{
	vs.LoadImageSequences("_TEST_DATA/july15-spinninghogue_0", "Intrinsics_Calib_", "Extrinsics_", "Color_", "Depth_", ".matte");

	int initial = 1;
	int amount = 5;
	int delta = 25;
	int trim_numerator = 1;
	int trim_denominator = 4;

	std::chrono::high_resolution_clock timer;

	std::vector<double> times;
	std::vector<int> voxel_dimensions;
	std::vector<int> voxel_trims;

	std::string data_filename = "data_output.txt";
	std::ofstream process_data;

	process_data.open(data_filename, std::ios::binary);

	if (!process_data.is_open())
	{
		std::cout << "Couldn't open file" << std::endl;
		return;
	}

	for (int i = initial; i < initial + amount; ++i)
	{
		MeshingVoxelParams mvp;

		int voxel_count = i * delta;

		mvp.center = Eigen::Vector3d(0, 0, 0);
		mvp.points_x = voxel_count + 1;
		mvp.points_y = voxel_count * 2 + 1;
		mvp.points_z = voxel_count + 1;
		mvp.voxel_size = 1.05 / (double)voxel_count;

		vs.SetVoxelGridParams(mvp);

		int trim = (voxel_count * trim_numerator) / trim_denominator;

		vs.SetTrim(trim + 1, trim * 2 + 1, trim + 1);

		std::string save_name = "FOURIER_" + std::to_string(voxel_count) + "_TRIM_" + std::to_string(trim);

		std::cout << "Processing " << save_name << "..." << std::endl;

		auto begin_time = timer.now();

		vs.SaveVolumeStream(save_name, SaveFileFormat::FOURIER);
		//vs.SaveVolumeStreamFourier(save_name);

		auto end_time = timer.now();

		auto delta_time = end_time - begin_time;

		double time_seconds = (double)delta_time.count() * 0.000000001;

		std::cout << time_seconds << " seconds elapsed for " << save_name << std::endl;

		times.push_back(time_seconds);
		voxel_dimensions.push_back(mvp.points_x);
		voxel_dimensions.push_back(mvp.points_y);
		voxel_dimensions.push_back(mvp.points_z);
		voxel_trims.push_back(vs.GetTrim().x());
		voxel_trims.push_back(vs.GetTrim().y());
		voxel_trims.push_back(vs.GetTrim().z());
	}

	for (int i = 0; i < amount; ++i)
	{
		process_data << times[i] << "\n";
		process_data << voxel_dimensions[3 * i] << "\n";
		process_data << voxel_dimensions[3 * i + 1] << "\n";
		process_data << voxel_dimensions[3 * i + 2] << "\n";
		process_data << voxel_trims[3 * i] << "\n";
		process_data << voxel_trims[3 * i + 1] << "\n";
		process_data << voxel_trims[3 * i + 2] << "\n";
		process_data << "\n";
	}

	process_data.close();
}

void TestMultiMesh::SaveGridZipIframes()
{
	MeshingVoxelParams mvp;

	int voxel_count = 100;

	mvp.center = Eigen::Vector3d(0, 0, 0);
	mvp.points_x = voxel_count + 1;
	mvp.points_y = 2 * voxel_count + 1;
	mvp.points_z = voxel_count + 1;
	mvp.voxel_size = 1.05 / (double)voxel_count;

	double threshold = 40000;
	double epsilon = 0.1;

	std::string save_name = "ZIPALG_" + std::to_string(voxel_count) + "_IFRAMETHRESH_" + std::to_string((int)threshold) + "_EPSILONTHSND_" + std::to_string((int)(epsilon * 1000));

	std::cout << "Loading images..." << std::endl;
	//vs.LoadImageSequences("_TEST_DATA/_TestingNewExtrinsics", "Intrinsics_Calib_", "Extrinsics_", "Color_", "Depth_", "Matte_");
	vs.LoadImageSequences("_TEST_DATA/july15-spinninghogue_0", "Intrinsics_Calib_", "Extrinsics_", "Color_", "Depth_", ".matte");
	vs.SetIFrameThreshold(threshold);
	vs.SetFrameCulling(epsilon);

	vs.SetVoxelGridParams(mvp);

	std::cout << "Saving grid..." << std::endl;
	vs.SaveVolumeStream(save_name, SaveFileFormat::ZIP);
	//vs.SaveVolumeStreamZipAlgorithm(save_name);
	std::cout << "Saved!" << std::endl;
}

void TestMultiMesh::SaveGridFourierIframes()
{
}
