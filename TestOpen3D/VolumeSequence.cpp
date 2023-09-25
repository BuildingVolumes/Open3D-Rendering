#include "VolumeSequence.h"

#include <iostream>
#include <vector>
#include "AdditionalUtilities.h"

VolumeSequence::VolumeSequence()
{
	
}

VolumeSequence::~VolumeSequence()
{
	
}

bool VolumeSequence::SetVoxelGridParams(MeshingVoxelParams mvp)
{
	this->mvp = mvp;

	return true;
}

/// <summary>
/// Adds all images to the mesh sequence
/// </summary>
/// <param name="root_folder"></param>
/// <param name="extrinsics_file_name"></param>
/// <param name="intrinsics_file_name"></param>
/// <param name="color_handle"></param>
/// <param name="depth_handle"></param>
/// <param name="matte_handle"></param>
/// <returns></returns>
int VolumeSequence::LoadImageSequences(std::string root_folder, std::string intrinsics_handle, std::string extrinsics_handle, std::string color_handle, std::string depth_handle, std::string matte_handle, float FPS)
{
	if (cm.IsLoaded())
	{
		cm.Unload();
	}

	cm.LoadTypeLivescan();

	int loaded_directories = 0;

	auto directories = GetDirectories(root_folder);

	std::vector<std::string> filename_pieces;

	for (int i = 0; i < directories.size(); ++i)
	{
		std::cout << directories[i] << std::endl;

		std::string intrinsic_file = "";
		std::string extrinsic_file = "";

		auto files = GetFiles(directories[i]);

		for (int j = 0; j < files.size(); ++j)
		{
			SplitString(files[j], filename_pieces, "/\\", "");
			std::string main_name = filename_pieces.back();

			auto find_int = main_name.find(intrinsics_handle);
			auto find_ext = main_name.find(extrinsics_handle);

			if (find_int != std::string::npos)
			{
				intrinsic_file = files[j];
			}

			if (find_ext != std::string::npos)
			{
				extrinsic_file = files[j];
			}

			filename_pieces.clear();
		}

		if (intrinsic_file != "" && extrinsic_file != "")
		{
			cm.AddCameraLivescan(directories[i], intrinsic_file, extrinsic_file, color_handle, depth_handle, matte_handle, FPS);
			++loaded_directories;
		}
	}

	return loaded_directories;
}

int VolumeSequence::SaveVolumeStream(std::string filename_without_extension, SaveFileFormat format, int start_frame, int end_frame)
{
	std::string filename = filename_without_extension;

	int frames_per_debug = 100;

	switch (format)
	{
	case SaveFileFormat::FOURIER:
		filename += file_ending_fourier;
		break;
	case SaveFileFormat::ZIP:
		filename += file_ending_zip;
		break;
	default:
		std::cout << "Invalid format!" << std::endl;
		return 0;
	}

	bool has_threshold = (threshold > 0);
	int has_threshold_int = has_threshold ? 1 : 0;
	
	std::cout << "Has Threshold: " << has_threshold_int << std::endl;

	int is_Iframe = 0;

	if (has_threshold)
	{
		filename += file_ending_Iframe;
	}
	else
	{
		filename += file_ending_flat;
	}

	std::ofstream savefile;

	savefile.open(filename, std::ios::binary);

	if (!savefile.is_open())
	{
		std::cout << "Couldn't open file" << std::endl;
		return 0;
	}

	int frames_processed = 0;
	int iframe_count = 0;
	int cam_max = cm.GetPlayableFrameCount();

	end_frame = cam_max < end_frame ? cam_max : end_frame;

	savefile.write(reinterpret_cast<char*>(&start_frame), sizeof(int));
	savefile.write(reinterpret_cast<char*>(&end_frame), sizeof(int));

	savefile.write(reinterpret_cast<char*>(&has_threshold_int), sizeof(int));

	std::shared_ptr<MeshingVoxelGrid> mvg = std::make_shared<MeshingVoxelGrid>();
	mvg->SetData(mvp);

	mvg->SaveMVP(savefile);

	std::shared_ptr<MeshingVoxelGrid> mvg_storage = std::make_shared<MeshingVoxelGrid>();

	if (has_threshold)
	{
		mvg_storage->SetData(mvp);
	}

	std::cout << "Saving from " << std::to_string(start_frame) << " to " << std::to_string(end_frame) << std::endl;

	for (int i = start_frame; i < end_frame; ++i)
	{
		if (i % frames_per_debug == 0)
		{
			std::cout << "Saving frame " << std::to_string(i) << " of " << filename << "..." << std::endl;
		}

		++frames_processed;

		cm.AllCamerasSeekFrame(i);
		cm.PackNewVoxelGrid(&(*mvg));

		//if (i == 0)
		//{
		//	mvg_storage->CopyGrid(*mvg);
		//}

		//auto mvg = cm.GetNewVoxelGrid(mvp);
		//mvg->SaveGridFourierToBinaryFile(savefile, trim.x(), trim.y(), trim.z());

		if (has_threshold)
		{
			auto dif = mvg_storage->AbsoluteDifference(*mvg);
			//auto sum = mvg_storage->Sum();

			//std::cout << "Difference (" << std::to_string(i) << "): " << dif << ", Sum: " << sum << std::endl;

			if (dif > threshold)
			{
				is_Iframe = 1;
				mvg_storage->CopyGrid(*mvg);
				++iframe_count;
			}
			else
			{
				mvg->Subtract(*mvg_storage);
				mvg->CullEpsilon(culling_value);
				is_Iframe = 0;
			}

			savefile.write(reinterpret_cast<char*>(&is_Iframe), sizeof(int));
		}

		switch (format)
		{
		case SaveFileFormat::FOURIER:
			mvg->SaveGridFourierBinaryWithoutMVP(savefile, trim.x(), trim.y(), trim.z());
			break;
		case SaveFileFormat::ZIP:
			mvg->SaveGridZipBinaryWithoutMVP(savefile);
			break;
		}

		mvg->FlushGrid();
	}

	savefile.close();

	if (has_threshold)
	{
		std::cout << iframe_count << " iframes created" << std::endl;
	}

	return frames_processed;
}

//int VolumeSequence::SaveVolumeStreamFourier(std::string filename, int start_frame, int end_frame)
//{
//	std::ofstream savefile;
//
//	savefile.open(filename, std::ios::binary);
//
//	if (!savefile.is_open())
//	{
//		std::cout << "Couldn't open file" << std::endl;
//		return 0;
//	}
//
//	int frames_processed = 0;
//	int cam_max = cm.GetPlayableFrameCount();
//
//	end_frame = cam_max < end_frame ? cam_max : end_frame;
//
//	savefile.write(reinterpret_cast<char*>(&start_frame), sizeof(int));
//	savefile.write(reinterpret_cast<char*>(&end_frame), sizeof(int));
//
//	std::shared_ptr<MeshingVoxelGrid> mvg = std::make_shared<MeshingVoxelGrid>();
//	mvg->SetData(mvp);
//
//	mvg->SaveMVP(savefile);
//
//	std::cout << "Saving from " << std::to_string(start_frame) << " to " << std::to_string(end_frame) << std::endl;
//
//	for (int i = start_frame; i < end_frame; ++i)
//	{
//		if (i % 100 == 0)
//		{
//			std::cout << "Saving frame " << std::to_string(i) << " of " << filename << "..." << std::endl;
//		}
//
//		++frames_processed;
//
//		cm.AllCamerasSeekFrame(i);
//		cm.PackNewVoxelGrid(&(*mvg));
//		//auto mvg = cm.GetNewVoxelGrid(mvp);
//		//mvg->SaveGridFourierToBinaryFile(savefile, trim.x(), trim.y(), trim.z());
//		mvg->SaveGridFourierBinaryWithoutMVP(savefile, trim.x(), trim.y(), trim.z());
//		mvg->FlushGrid();
//	}
//
//	savefile.close();
//	return frames_processed;
//}

int VolumeSequence::LoadVolumeStreamAndExtractMeshes(std::string filename)
{
	std::ifstream savefile;

	savefile.open(filename, std::ios::binary);

	if (!savefile.is_open())
	{
		std::cout << "Couldn't open file" << std::endl;
		return 0;
	}

	int frames_processed = 0;

	int start_frame;
	int end_frame;

	savefile.read(reinterpret_cast<char*>(&start_frame), sizeof(int));
	savefile.read(reinterpret_cast<char*>(&end_frame), sizeof(int));

	MeshingVoxelGrid mvg;

	int step = 20;

	for (int i = start_frame; i < end_frame; ++i)
	{
		std::cout << "Loading frame " << std::to_string(i) << std::endl;

		++frames_processed;

		mvg.ReadGridFourierFromBinary(savefile);

		auto mesh = mvg.ExtractMesh();

		if (i % step == 0)
		{
			DrawObject(*mesh);
		}
	}

	savefile.close();

	return frames_processed;
}

//int VolumeSequence::SaveVolumeStreamZipAlgorithm(std::string filename, int start_frame, int end_frame)
//{
//	std::ofstream savefile;
//
//	savefile.open(filename, std::ios::binary);
//
//	if (!savefile.is_open())
//	{
//		std::cout << "Couldn't open file" << std::endl;
//		return 0;
//	}
//
//	int frames_processed = 0;
//	int cam_max = cm.GetPlayableFrameCount();
//
//	end_frame = cam_max < end_frame ? cam_max : end_frame;
//
//	savefile.write(reinterpret_cast<char*>(&start_frame), sizeof(int));
//	savefile.write(reinterpret_cast<char*>(&end_frame), sizeof(int));
//
//	std::shared_ptr<MeshingVoxelGrid> mvg = std::make_shared<MeshingVoxelGrid>();
//	mvg->SetData(mvp);
//
//	mvg->SaveMVP(savefile);
//
//	std::cout << "Saving from " << std::to_string(start_frame) << " to " << std::to_string(end_frame) << std::endl;
//
//	for (int i = start_frame; i < end_frame; ++i)
//	{
//		if (i % 100 == 0)
//		{
//			std::cout << "Saving frame " << std::to_string(i) << " of " << filename << "..." << std::endl;
//		}
//
//		++frames_processed;
//
//		cm.AllCamerasSeekFrame(i);
//		cm.PackNewVoxelGrid(&(*mvg));
//		mvg->SaveGridZipBinaryWithoutMVP(savefile);
//		mvg->FlushGrid();
//	}
//
//	savefile.close();
//	return frames_processed;
//}
