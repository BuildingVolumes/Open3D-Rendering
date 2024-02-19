#include "VolumeSequence.h"

#include <iostream>
#include <vector>
#include "AdditionalUtilities.h"

#define DEBUG_VOLUME_SEQUENCE 0
//#define DEBUG_VOLUME_SEQUENCE 1

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
	std::vector<int> frame_locations;

	int byte_count = 0;

	std::string filename = filename_without_extension;

	int frames_per_debug = 50;

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

	byte_count += 3 * sizeof(int);

	int frame_bytes_start = byte_count;

	for (int i = start_frame; i < end_frame; ++i)
	{
		frame_locations.push_back(0);
		savefile.write(reinterpret_cast<char*>(&frame_locations[i - start_frame]), sizeof(int));
		byte_count += sizeof(int);
	}

	int frame_bytes_end = byte_count;

	std::shared_ptr<MeshingVoxelGrid> mvg = std::make_shared<MeshingVoxelGrid>();
	mvg->SetData(mvp);

	std::cout << "Params: \n" <<
		start_frame << ", " << end_frame << "\n" <<
		mvp.points_x << ", " << mvp.points_y << ", " << mvp.points_z << "\n" <<
		mvp.voxel_size << "\n" <<
		mvp.center << std::endl;

	byte_count += mvp.SaveToFile(savefile); //mvg->SaveMVP(savefile);

	std::shared_ptr<MeshingVoxelGrid> mvg_storage = std::make_shared<MeshingVoxelGrid>();

	if (has_threshold)
	{
		mvg_storage->SetData(mvp);
	}

	std::cout << "Saving from " << std::to_string(start_frame) << " to " << std::to_string(end_frame) << std::endl;

	for (int i = start_frame; i < end_frame; ++i)
	{
#if DEBUG_VOLUME_SEQUENCE
		if (i % frames_per_debug == 0)
		{
			std::cout << "Saving frame " << std::to_string(i) << " of " << filename << "..." << std::endl;
		}
#endif

		++frames_processed;

		frame_locations[i - start_frame] = byte_count;

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
			byte_count += sizeof(int);
		}

		switch (format)
		{
		case SaveFileFormat::FOURIER:
			byte_count += mvg->SaveGridFourierBinaryWithoutMVP(savefile, trim.x(), trim.y(), trim.z());
			break;
		case SaveFileFormat::ZIP:
			byte_count += mvg->SaveGridZipBinaryWithoutMVP(savefile);
			break;
		}

		//frame_locations[i - start_frame] = byte_count;
#if DEBUG_VOLUME_SEQUENCE
		if (i % frames_per_debug == 0)
		{
			std::cout << "BYTE LOCATION OF FRAME " << i << ": " << frame_locations[i - start_frame] << std::endl;
		}

#endif
		mvg->FlushGrid();
	}

	int file_end = byte_count;

	savefile.seekp(frame_bytes_start);

	for (int i = start_frame; i < end_frame; ++i)
	{
		savefile.write(reinterpret_cast<char*>(&frame_locations[i - start_frame]), sizeof(int));
		//std::cout << "Saved Frame " << i << ": " << frame_locations[i - start_frame] << std::endl;
	}

	savefile.seekp(file_end);

	std::cout << "FILE END: " << file_end << ", ACCORDING TO FILE: " << savefile.tellp() << std::endl;

	savefile.close();

	if (has_threshold)
	{
		std::cout << iframe_count << " iframes created" << std::endl;
	}

	return frames_processed;
}

void VolumeSequence::LoadVolumeStream(std::string filename_with_extension)
{
	std::vector<std::string> filename_and_ext;

	SplitString(filename_with_extension, filename_and_ext, ".");

	if (filename_and_ext[1][2] == 'z')
	{
		decode_type = SaveFileFormat::ZIP;
	}
	else if (filename_and_ext[1][2] == 'f')
	{
		decode_type = SaveFileFormat::FOURIER;
	}
	else
	{
		std::cout << "Invalid file type!" << std::endl;
		return;
	}

	if (current_loaded_stream.is_open())
	{
		std::cout << "Stream already loaded!" << std::endl;
		decode_type = SaveFileFormat::ERROR_NO_TYPE;
		return;
	}

	current_loaded_stream.open(filename_with_extension, std::ios::binary);

	if (!current_loaded_stream.is_open())
	{
		std::cout << "Couldn't open file" << std::endl;
		decode_type = SaveFileFormat::ERROR_NO_TYPE;
		return;
	}

	int byte_count = 0;

	int has_threshold_int = 0;

	int isIframe = 0;

	current_loaded_stream.read(reinterpret_cast<char*>(&loaded_sequence_start), sizeof(int));
	current_loaded_stream.read(reinterpret_cast<char*>(&loaded_sequence_end), sizeof(int));

	current_loaded_stream.read(reinterpret_cast<char*>(&has_threshold_int), sizeof(int));

	int frame_count = loaded_sequence_end - loaded_sequence_start;

	for (int i = loaded_sequence_start; i < loaded_sequence_end; ++i)
	{
		loaded_frame_locations.push_back(0);
		current_loaded_stream.read(reinterpret_cast<char*>(&loaded_frame_locations[i - loaded_sequence_start]), sizeof(int));
		byte_count += sizeof(int);
	
		//std::cout << "Frame " << i << ": " << loaded_frame_locations[i - loaded_sequence_start] << std::endl;
	}

	byte_count += 3 * sizeof(int);

	byte_count += mvp.LoadFromFile(current_loaded_stream);

	//std::cout << "Params: \n" << 
	//	loaded_sequence_start << ", " << loaded_sequence_end << "\n" <<
	//	mvp.points_x << ", " << mvp.points_y << ", " << mvp.points_z << "\n" << 
	//	mvp.voxel_size << "\n" << 
	//	mvp.center << std::endl;

	return;
}

std::shared_ptr<MeshingVoxelGrid> VolumeSequence::LoadFrameFromStream(int frame_num)
{
	auto to_return = std::make_shared<MeshingVoxelGrid>();

	int array_loc = frame_num - loaded_sequence_start;
	int frame_loc = loaded_frame_locations[array_loc];

	std::cout << "Loading at: " << frame_num << ", at file location: " << frame_loc << std::endl;


	to_return->SetData(mvp);

	switch (decode_type)
	{
	case SaveFileFormat::FOURIER:
		to_return->LoadGridFourierBinary(current_loaded_stream, frame_loc);
		break;

	case SaveFileFormat::ZIP:
		to_return->LoadGridZipBinary(current_loaded_stream, frame_loc);
		break;

	default:
		std::cout << "ERROR! Irregular format!" << std::endl;
		break;
	}

	return to_return;
}

void VolumeSequence::CloseLoadedVolumeStream()
{
	if (!current_loaded_stream.is_open())
	{
		std::cout << "No stream open!" << std::endl;
		return;
	}

	current_loaded_stream.close();
	loaded_frame_locations.clear();

	loaded_sequence_end = 0;
	loaded_sequence_start = 0;

	decode_type = SaveFileFormat::ERROR_NO_TYPE;
}

int VolumeSequence::SaveAllFramesAsMeshes(std::string root_folder, std::string main_file_name, std::string mesh_filename_without_extension)
{
	std::ofstream main_savefile;

	int framecount = cm.GetPlayableFrameCount();

	main_savefile.open(root_folder + "/" + main_file_name);

	for (int i = 0; i < framecount; ++i)
	{
		std::cout << "Saving Frame " << i << "..." << std::endl;

		std::string new_name = root_folder + "/" + mesh_filename_without_extension + "_" + GetNumberFixedLength(i, 8);// std::to_string(i);

		cm.AllCamerasSeekFrame(i);

		auto tri_mesh = cm.GetMeshUsingNewVoxelGrid(mvp, 0);

		std::cout << "verts: " << tri_mesh->HasVertices() << ", count: " << tri_mesh->vertices_.size() << std::endl;
		std::cout << "tris: " << tri_mesh->HasTriangles() << ", count: " << tri_mesh->triangles_.size() << std::endl;
		std::cout << "normals: " << tri_mesh->HasTriangleNormals() << ", count: " << tri_mesh->triangle_normals_.size() << std::endl;

		if (!open3d::io::WriteTriangleMeshToOBJ(new_name + ".obj", *tri_mesh, true, false, true, false, true, false))
		{
			system("pause");
			//std::cout << open3d::core::
		}

		//main_savefile.write(new_name.c_str(), new_name.size());
		main_savefile << new_name << "\n";
	}

	main_savefile.close();

	std::cout << "DONE!" << std::endl;
}

void VolumeSequence::SaveFrameAsMesh(std::string filename, int frame_number)
{
	//std::ofstream main_savefile;

	//main_savefile.open(filename);

	cm.AllCamerasSeekFrame(frame_number);

	//auto tri_mesh = cm.GetMeshUsingNewVoxelGrid(mvp, 0);

	MKV_Rendering::VoxelGridData vgd;

	vgd.voxel_size = mvp.voxel_size;
	//vgd.
	//vgd.signed_distance_field_truncation = 0.03f;

	auto tri_mesh = cm.GetMesh(&vgd).ToLegacyTriangleMesh();

	//std::cout << "verts: " << tri_mesh->HasVertices() << ", count: " << tri_mesh->vertices_.size() << std::endl;
	//std::cout << "tris: " << tri_mesh->HasTriangles() << ", count: " << tri_mesh->triangles_.size() << std::endl;
	//std::cout << "normals: " << tri_mesh->HasTriangleNormals() << ", count: " << tri_mesh->triangle_normals_.size() << std::endl;

	std::cout << "verts: " << tri_mesh.HasVertices() << ", count: " << tri_mesh.vertices_.size() << std::endl;
	std::cout << "tris: " << tri_mesh.HasTriangles() << ", count: " << tri_mesh.triangles_.size() << std::endl;
	std::cout << "normals: " << tri_mesh.HasTriangleNormals() << ", count: " << tri_mesh.triangle_normals_.size() << std::endl;

	//if (!open3d::io::WriteTriangleMeshToOBJ(filename, *tri_mesh, true, false, true, false, true, false))
	if (!open3d::io::WriteTriangleMeshToOBJ(filename, tri_mesh, true, false, true, false, true, false))
	{
		std::cout << "Open3D refused to write file" << std::endl;
		system("pause");
		//std::cout << open3d::core::
	}
	else
	{
		std::cout << "Triangle mesh was successfully written as: " << filename << std::endl;
	}
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
#if DEBUG_VOLUME_SEQUENCE
		std::cout << "Loading frame " << std::to_string(i) << std::endl;
#endif
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

std::shared_ptr<MeshingVoxelGrid> VolumeSequence::GetVoxelGridAtFrame(int frame)
{
	cm.AllCamerasSeekFrame(frame);
	return cm.GetNewVoxelGrid(mvp);
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
