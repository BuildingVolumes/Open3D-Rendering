#include "TestFFT.h"
#include <iostream>
#include <string>
#include <filesystem>
#include "CameraManager.h"
#include "VoxelGridData.h"

void TestFFT::print_signal(int num_count, fftwf_complex* signal)
{
	std::cout << "SIGNAL_START" << std::endl;

	for (int i = 0; i < num_count; ++i)
	{
		std::cout << std::to_string(i) << ": " << signal[i][real_num] << "\n";
	}

	std::cout << "SIGNAL_END" << std::endl;
}

void TestFFT::print_signal(int num_count, fftw_complex* signal)
{
	std::cout << "SIGNAL_START" << std::endl;

	for (int i = 0; i < num_count; ++i)
	{
		std::cout << std::to_string(i) << ": " << signal[i][real_num] << ", " << signal[i][imag_num] << "\n";
	}

	std::cout << "SIGNAL_END" << std::endl;
}

void TestFFT::basic_test()
{
	int num_points = 16;

	fftwf_complex* signal = new fftwf_complex[num_points];
	fftwf_complex* result = new fftwf_complex[num_points];

	fftwf_plan p = fftwf_plan_dft_1d(num_points, signal, result, FFTW_FORWARD, FFTW_ESTIMATE);

	for (int i = 0; i < num_points; ++i)
	{
		//Some random signal
		signal[i][real_num] = (i % 2 > 0) || (i % 3 > 0);
	}

	print_signal(num_points, signal);

	fftwf_execute(p);

	print_signal(num_points, result);

	std::cout << "I ran!" << std::endl;
}

void TestFFT::grid_extraction_test()
{
	double insignificant = 0.2;

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
	
	auto grid = cm.GetNewVoxelGridAtTimestamp(params, timestamp);
	//std::shared_ptr<MeshingVoxelGrid> grid = std::make_shared<MeshingVoxelGrid>(params);

	//grid->MakeDebuggingSphere(Eigen::Vector3d(0, 0, 0), 0.5);

	DrawObject(*grid->ExtractMesh());

	std::string savefile_name = "TestingSaveGrid.vgdf";
	std::string savefile_name2 = "TestingSaveGrid2.vgdf";
	std::string savefile_fourier = "TestingSaveFourier.vgff";

	//std::string savefile_fourier_41_81_41 = "TestingSaveFourier.vgff";


	//grid->SaveDenseGridToBinaryFile(savefile_name);
	grid->SaveGridFourierToBinaryFile(savefile_fourier, 41, 81, 41);


	grid->FlushGrid();
	
	std::cout << "\n\nReading grid..." << std::endl;
	
	grid->ReadGridFourierFromBinary(savefile_fourier);

	DrawObject(*grid->ExtractMesh());

	////grid->SaveDenseGridToPlaintextFile(savefile_name2);
	//
	//grid->FlushGrid();
	//
	//grid->ReadDenseGridFromBinaryFile(savefile_name);
	//
	//DrawObject(*grid->ExtractMesh());

	return;


	int loc = 0;

	//for (int x = 0; x < params.points_x; ++x)
	//{
	//	for (int y = 0; y < params.points_y; ++y)
	//	{
	//		for (int z = 0; z < params.points_z; ++z, ++loc)
	//		{
	//			grid->SetGridValue(i, abs(std::clamp(result[i][real_num], -1.0, 1.0)));
	//			grid->SetGridType(i, (result[i][real_num] > 0) ? MeshingVoxelType::SOLID : MeshingVoxelType::AIR);
	//		}
	//	}
	//}

	//auto mesh = cm.GetMeshUsingNewVoxelGridAtTimestamp(params, 0, timestamp);

	//DrawObject(*grid->ExtractMesh());

	//TODO: Deprecate UVPackmaster
	//Yes. Really. Good luck soldier.

	fftw_complex* signal = new fftw_complex[p_tot];
	fftw_complex* result = new fftw_complex[p_tot];

	std::cout << "Executing Fourier Transform..." << std::endl;

	auto fourier_plan = fftw_plan_dft_3d(params.points_x, params.points_y, params.points_z, signal, result, FFTW_FORWARD, FFTW_ESTIMATE);
	//auto fourier_plan = fftw_plan_dft_1d(params.points_x * params.points_y * params.points_z, signal, result, FFTW_FORWARD, FFTW_ESTIMATE);

	for (int i = 0; i < p_tot; ++i)
	{
		double sign = (grid->GetGridType(i) == MeshingVoxelType::SOLID) ? 1 : -1;

		auto gridval = grid->GetGridValue(i);

		//if (gridval == 1)
		//{
		//	gridval = 2 * gridval;
		//}

		signal[i][real_num] = gridval * sign;
	}

	fftw_execute(fourier_plan);

	//for (int i = 0; i < p_tot; ++i)
	//{
	//	grid->SetGridValue(i, std::min(abs(result[i][real_num]), 1.0));
	//  grid->SetGridValue(i, abs(result[i][real_num]));
	//	grid->SetGridType(i, (result[i][real_num] > 0) ? MeshingVoxelType::SOLID : MeshingVoxelType::AIR);
	//}
	//
	//std::cout << "extracting mesh" << std::endl;
	//
	//DrawObject(*grid->ExtractMesh());

	int unimportant = 0;
	int unimportant_real = 0;
	int important = 0;
	int important_real = 0;

	for (int i = 0; i < p_tot; ++i)
	{
		bool insig_real = abs(result[i][real_num]) < insignificant;
		bool insig_imag = abs(result[i][imag_num]) < insignificant;

		unimportant += (insig_real && insig_imag);
		unimportant_real += (insig_real);
		important = (!insig_real || !insig_imag);
		important_real += (!insig_real);
	}

	std::cout << "Cutting frequencies..." << std::endl;

	//TODO: Load Volumetric Video in full, create video where freqency culling is variable throughout

	loc = 0;
	
	int cut_freq = 0;

	int keep_corners = 50;
	
	for (int x = 0; x < params.points_x; ++x)
	{
		for (int y = 0; y < params.points_y; ++y)
		{
			for (int z = 0; z < params.points_z; ++z, ++loc)
			{
				bool x_in_pad_range = x < keep_corners || params.points_x - x <= keep_corners;
				bool y_in_pad_range = y < keep_corners || params.points_y - y <= keep_corners;
				bool z_in_pad_range = z < keep_corners || params.points_z - z <= keep_corners;
	
				if (!x_in_pad_range || !y_in_pad_range || !z_in_pad_range)
				{
					++cut_freq;

					result[loc][real_num] = 0;
					result[loc][imag_num] = 0;
				}
			}
		}
	}

	std::cout << "Cut: " << cut_freq << "/" << loc << ", " << ((double)cut_freq / (double)loc) <<  std::endl;

	//int pad = 10000;
	//
	//for (int loc = 0; loc < p_tot; ++loc)
	//{
	//	int shifted_loc = (loc + p_tot / 2) % p_tot;
	//	bool is_in_pad_range = shifted_loc < pad || p_tot - shifted_loc <= pad;
	//
	//	if (is_in_pad_range)
	//	{
	//		result[loc][real_num] = 0;
	//		result[loc][imag_num] = 0;
	//	}
	//}

	std::cout <<
		"Unimportant: " << unimportant << "/" << p_tot << "\t" <<
		"Unimportant Real: " << unimportant_real << "/" << p_tot << "\t" <<
		"Important: " << important << "/" << p_tot << "\t" <<
		"Important Real: " << important_real << "/" << p_tot << std::endl;

	auto inverse_plan = fftw_plan_dft_3d(params.points_x, params.points_y, params.points_z, result, signal, FFTW_BACKWARD, FFTW_ESTIMATE);
	//auto inverse_plan = fftw_plan_dft_1d(params.points_x * params.points_y * params.points_z, result, signal, FFTW_BACKWARD, FFTW_ESTIMATE);

	fftw_execute(inverse_plan);

	for (int i = 0; i < p_tot; ++i)
	{
		//grid->SetGridValue(i, std::min(abs(signal[i][real_num]), 1.0));
		grid->SetGridValue(i, abs(signal[i][real_num]));
		grid->SetGridType(i, (signal[i][real_num] > 0) ? MeshingVoxelType::SOLID : MeshingVoxelType::AIR);
	}

	auto test_mesh = grid->ExtractMesh();

	DrawObject(*test_mesh);

	auto test_image = cm.CreateUVMapAndTexture(&(*test_mesh), false);

	//auto tex = cm.CreateUVMapAndTexture(&(*mesh), false);
	//
	//std::cout << mesh->HasTriangleUvs() <<  ", " << mesh->triangle_uvs_.size() <<  ", " << mesh->triangles_.size() << std::endl;
	//
	//system("pause");
	//
	//open3d::io::WriteTriangleMeshToOBJ("TestNewGrid.obj", *mesh, true, false, true, false, false, false);
	//open3d::io::WriteImageToPNG("TestNewGrid.png", *tex);

	//auto grid_hashmap = grid.GetBlockHashmap();

	//auto valbuff = grid_hashmap->GetValueBuffer();

	//auto sizes = valbuff.NumDims();

	WriteOBJ("Hogue_" + the_date + ".obj", "", &(*test_mesh));
	open3d::io::WriteImageToPNG("Hogue_" + the_date + ".png", *test_image);
}

void TestFFT::run(int argc, char** argv)
{
	//basic_test();
	grid_extraction_test();
	//FourierTesting();
}

void TestFFT::FourierTesting()
{
	auto len = 15;
	auto p_tot = len;

	fftw_complex* signal = new fftw_complex[p_tot];
	fftw_complex* result = new fftw_complex[p_tot];


	for (int i = 0; i < p_tot; ++i)
	{
		signal[i][real_num] = 2 * (i % 2) - 1;
		signal[i][imag_num] = 0;
	}

	auto fourier_plan = fftw_plan_dft_1d(p_tot, signal, result, FFTW_FORWARD, FFTW_ESTIMATE);

	print_signal(p_tot, signal);

	fftw_execute(fourier_plan);

	print_signal(p_tot, result);
}

void TestFFT::WriteOBJ(std::string filename, std::string filepath, open3d::geometry::TriangleMesh* mesh)
{
	std::ofstream writer;

	if (filepath != "")
	{
		std::filesystem::create_directories(filepath);

		writer.open(filepath + "/" + filename);
	}
	else
	{
		writer.open(filename);
	}

	for (int i = 0; i < mesh->vertices_.size(); ++i)
	{
		auto vert = mesh->vertices_[i];

		writer << "v " << vert.x() << " " << vert.y() << " " << vert.z() << "\n";
	}

	for (int i = 0; i < mesh->triangle_uvs_.size(); ++i)
	{
		auto uv = mesh->triangle_uvs_[i];

		writer << "vt " << uv.x() << " " << uv.y() << "\n";
	}

	for (int i = 0; i < mesh->vertex_normals_.size(); ++i)
	{
		auto norm = mesh->vertex_normals_[i];

		writer << "vn " << norm.x() << " " << norm.y() << " " << norm.z() << "\n";
	}

	for (int i = 0; i < mesh->triangles_.size(); ++i)
	{
		auto tri = mesh->triangles_[i];

		if (mesh->triangle_uvs_.size() > 0)
		{
			writer << "f " <<
				(tri.x() + 1) << "/" << (tri.x() + 1) << "/" << (tri.x() + 1) << " " <<
				(tri.y() + 1) << "/" << (tri.y() + 1) << "/" << (tri.y() + 1) << " " <<
				(tri.z() + 1) << "/" << (tri.z() + 1) << "/" << (tri.z() + 1) << "\n";
		}
		else
		{
			writer << "f " <<
				(tri.x() + 1) << "//" << (tri.x() + 1) << " " <<
				(tri.y() + 1) << "//" << (tri.y() + 1) << " " <<
				(tri.z() + 1) << "//" << (tri.z() + 1) << "\n";
		}
	}

	writer.close();
}
