#include "CameraManager.h"
#include "AdditionalUtilities.h"
#include "ErrorLogger.h"

#include "MKV_Data.h"
#include "Image_Data.h"
#include "Livescan_Data.h"
#include "TextureUnpacker.h"

#include <fstream>

using namespace MKV_Rendering;

void MKV_Rendering::CameraManager::CauseError(bool cause_abort)
{
	bool bad_thing_happened = true;

	if (bad_thing_happened)
	{
		ErrorLogger::LOG_ERROR("DONT WORRY! This error is suppsoed to happen here!", cause_abort);
	}
}

void MKV_Rendering::CameraManager::LoadStructure(std::string structure_path, std::map<std::string, std::string> *data)
{
	std::fstream structure_file;
	std::vector<std::string> lines;
	std::string parser;

	structure_file.open(structure_path);

	if (!structure_file.is_open())
	{
		ErrorLogger::LOG_ERROR("No suitable structure file found at " + structure_path + "!", true);
	}

	while (std::getline(structure_file, parser))
	{
		if (parser != "")
		{
			lines.push_back(parser);

			parser = "";
		}
	}

	for (auto line : lines)
	{
		std::vector<std::string> name_and_value;
		SplitString(line, name_and_value, ' ');
		(*data)[name_and_value[0]] = name_and_value[1];
	}

	structure_file.close();
}

CameraManager::CameraManager()
{
	
}

bool MKV_Rendering::CameraManager::LoadTypeStructure(std::string root_folder, std::string structure_file_name)
{
	if (loaded)
	{
		ErrorLogger::LOG_ERROR("Manager already loaded!");
		return false;
	}

	std::vector<std::string> all_folders = GetDirectories(root_folder);
	//open3d::utility::filesystem::ListFilesInDirectory(root_folder, all_folders);
	
	std::map<std::string, std::string> camera_structure;

	for (auto _folder : all_folders)
	{
		std::string structure_path = _folder + "/" + structure_file_name;

		ErrorLogger::EXECUTE("Load File Structure", this, &CameraManager::LoadStructure, structure_path, &camera_structure);

		std::cout << "Camera specifics at " << _folder << ": " << std::endl;
		for (auto _pair : camera_structure)
		{
			std::cout << "\t" << _pair.first << ": '" << _pair.second << "'" << std::endl;
		}

		std::string c_type = camera_structure["Type"];

		if (c_type == "mkv")
		{
			camera_data.push_back(new MKV_Data(
				_folder,
				camera_structure["MKV_File"],
				camera_structure["Calibration_File"]
			));
		}
		else if (c_type == "image")
		{
			camera_data.push_back(new Image_Data(
				_folder,
				camera_structure["Color"],
				camera_structure["Depth"],
				camera_structure["Intrinsics_Json"],
				camera_structure["Calibration_File"],
				camera_structure["FPS"]
			));
		}
		else
		{
			ErrorLogger::LOG_ERROR("Unrecognized file format, " + c_type + "!");
		}

		camera_structure.clear();

		std::cout << "Finished initializing camera!\n" << std::endl;
	}

	if (camera_data.size() == 0)
	{
		ErrorLogger::LOG_ERROR(
			"No data files present!"
		);

		return false;
	}

	loaded = true;

	return true;
}

bool MKV_Rendering::CameraManager::LoadTypeLivescan(std::string image_root_folder, std::string matte_root_folder, float FPS)
{
	if (loaded)
	{
		ErrorLogger::LOG_ERROR("Manager already loaded!");
		return false;
	}

	std::vector<std::string> extrinsic_file_candidates;
	std::string extrinsic_file = "";

	open3d::utility::filesystem::ListFilesInDirectory(image_root_folder, extrinsic_file_candidates);

	std::cout << image_root_folder << std::endl;

	for (int i = 0; i < extrinsic_file_candidates.size(); ++i)
	{
		std::vector<std::string> file_name_and_ext;

		SplitString(extrinsic_file_candidates[i], file_name_and_ext, '.');

		std::cout << extrinsic_file_candidates[i] << std::endl;

		if (file_name_and_ext.back() == "log")
		{
			extrinsic_file = extrinsic_file_candidates[i];
			break;
		}
	}

	if (extrinsic_file == "")
	{
		E_LOG("No extrinsics file detected!", false);

		return false;
	}

	std::vector<std::string> all_folders = GetDirectories(image_root_folder);
	std::vector<std::string> matte_folders;

	if (matte_root_folder != "")
	{
		std::cout << "Matte found: " << matte_root_folder << std::endl;
		matte_folders = GetDirectories(matte_root_folder);
	}

	std::fstream extrinsics_filestream;
	extrinsics_filestream.open(extrinsic_file);
	std::vector<std::string> lines;

	std::map<std::string, std::string> camera_structure;

	for (int i = 0; i < all_folders.size(); ++i)
	{
		std::string parser = "";

		auto _folder = all_folders[i];

		std::cout << _folder << std::endl;

		std::vector<std::string> extrinsics_string;

		if (!std::getline(extrinsics_filestream, parser))
		{
			break;
		}
		else if (parser == "")
		{
			break;
		}

		for (int i = 0; i < 4; ++i)
		{
			std::getline(extrinsics_filestream, parser);
			extrinsics_string.push_back(parser);
		}

		std::string _matte = "";

		if (matte_root_folder != "")
		{
			_matte = matte_folders[i];
		}

		camera_data.push_back(new Livescan_Data(
			_folder, _matte,
			extrinsics_string,
			FPS
		));
	}

	if (camera_data.size() == 0)
	{
		ErrorLogger::LOG_ERROR(
			"No data files present!"
		);

		return false;
	}

	loaded = true;

	return true;
}

CameraManager::~CameraManager()
{
	if (loaded)
	{
		Unload();
	}
}

bool MKV_Rendering::CameraManager::Unload()
{
	if (!loaded)
	{
		ErrorLogger::LOG_ERROR("Attempting to unload manager that has not been loaded!");
		return false;
	}

	while (!camera_data.empty())
	{
		if (camera_data.back() != nullptr)
		{
			delete camera_data.back();
		}

		camera_data.pop_back();
	}

	loaded = false;

	return true;
}

open3d::t::geometry::TriangleMesh MKV_Rendering::CameraManager::GetMesh(VoxelGridData* data)
{
	return ErrorLogger::EXECUTE("Construct Voxel Grid", this, &MKV_Rendering::CameraManager::GetVoxelGrid, data).ExtractSurfaceMesh(0.0f);
}

open3d::geometry::VoxelGrid MKV_Rendering::CameraManager::GetOldVoxelGrid(VoxelGridData *data)
{
	open3d::geometry::VoxelGrid grid;

	grid.voxel_size_ = data->voxel_size;

	for (auto cam : camera_data)
	{
		ErrorLogger::EXECUTE("Pack Frame into Voxel Grid", cam, &Abstract_Data::PackIntoOldVoxelGrid, &grid);
	}

	return grid;
}

open3d::t::geometry::TSDFVoxelGrid MKV_Rendering::CameraManager::GetVoxelGrid(VoxelGridData* data)
{
	open3d::core::Device device(data->device_code);

	open3d::t::geometry::TSDFVoxelGrid voxel_grid(

		{
			{"tsdf", open3d::core::Dtype::Float32},
			{"weight", open3d::core::Dtype::UInt16},
			{"color", open3d::core::Dtype::UInt16}
		},

		data->voxel_size, data->signed_distance_field_truncation,
		16, data->blocks, device
	);

	for (auto cam : camera_data)
	{
		ErrorLogger::EXECUTE("Pack Frame into Voxel Grid", cam, &Abstract_Data::PackIntoVoxelGrid, &voxel_grid, data);
	}

	return voxel_grid;
}

std::shared_ptr<open3d::geometry::Image> MKV_Rendering::CameraManager::CreateUVMapAndTexture(open3d::geometry::TriangleMesh* mesh, bool useTheBadTexturingMethod)//, float depth_epsilon)
{
	//If there are no cameras, throw an error
	if (camera_data.size() <= 0)
	{
		ErrorLogger::LOG_ERROR("No cameras present!", true);
	}
	
	std::vector<open3d::geometry::Image> color_images;
	std::vector<open3d::geometry::Image> depth_images;

	//Write all camera images at the current time to one vector
	for (auto cam : camera_data)
	{
		auto rgbd_im = ErrorLogger::EXECUTE("Get RGBD Image for Texture Stitching", cam, &Abstract_Data::GetFrameRGBD);

		color_images.push_back(rgbd_im->color_);
		depth_images.push_back(*(rgbd_im->depth_.ConvertDepthToFloatImage()));
	}



	int camera_count = camera_data.size();	
	int vert_count = mesh->vertices_.size();

	//Save data about the cameras for future use, and allocate triangle vectors

	std::vector<std::vector<int>> camera_triangles;
	std::vector<Eigen::Vector3d> camera_positions_original;
	std::vector<Eigen::Vector3d> camera_positions_rotated;
	std::vector<Eigen::Vector3d> camera_positions_normalized;
	std::vector<Eigen::Matrix3d> camera_rotations;
	std::vector<Eigen::Matrix3d> camera_intrinsics;

	mesh->textures_.clear();

	for (int i = 0; i < camera_count; ++i)
	{
		auto mat = camera_data[i]->GetExtrinsicMat();

		mesh->textures_.push_back(color_images[i]);

		camera_triangles.push_back(std::vector<int>());
		camera_positions_original.push_back(mat.block<3, 1>(0, 3));
		camera_rotations.push_back(mat.block<3, 3>(0, 0));
		camera_positions_rotated.push_back(mat.block<3, 3>(0, 0) * mat.block<3, 1>(0, 3));
		camera_positions_normalized.push_back(camera_positions_rotated[i].normalized());
		camera_intrinsics.push_back(camera_data[i]->GetIntrinsicMat());
	}

	//Used to allow an easy way to modify triangle indices
	std::vector<int> index_redirect;

	mesh->triangle_material_ids_.resize(mesh->triangles_.size());

	//Number of triangles without UVs
	int withoutUV = 0;

	//Iterate through all the triangles and find the normal that aligns closest to a camera
	for (int i = 0; i < mesh->triangles_.size(); ++i)
	{
		double highest_dot = -2.0;
		double lowest_depth_delta = DBL_MAX;
		int best_camera = -1;

		//Currently each index should point to itself, we will force it to point elsewhere if need be
		index_redirect.push_back(i);

		//mesh->vertex_colors_[mesh->triangles_[i](0)] = Eigen::Vector3d(1, 0, 0);
		//mesh->vertex_colors_[mesh->triangles_[i](1)] = Eigen::Vector3d(1, 0, 0);
		//mesh->vertex_colors_[mesh->triangles_[i](2)] = Eigen::Vector3d(1, 0, 0);

		for (int j = 0; j < camera_count; ++j)
		{
			//Take the depth that is most facing the image
			auto normal = (mesh->vertices_[mesh->triangles_[i](0)] - mesh->vertices_[mesh->triangles_[i](1)]).cross(
				mesh->vertices_[mesh->triangles_[i](0)] - mesh->vertices_[mesh->triangles_[i](2)]
			).normalized();

			auto normal_dot = normal.dot(-camera_positions_normalized[j]);

			double depth_delta = 0;

			bool bad_camera = false;
			for (int k = 0; k < 3; ++k)
			{
				//Check that the UVs would actually be on the image
				Eigen::Vector3d uvz = camera_intrinsics[j] *
					(camera_rotations[j] * mesh->vertices_[mesh->triangles_[i](k)] + camera_positions_original[j]);

				uvz.x() /= (uvz.z());

				uvz.y() /= (uvz.z());

				double depth = 0;
				bool in_bounds;
				std::tie(in_bounds, depth) = depth_images[j].FloatValueAt(uvz.x(), uvz.y());

				//float delta_depth = uvz.z() - depth;

				//Cull if the depth is OOB
				if (!in_bounds)// || (delta_depth * delta_depth > depth_epsilon * depth_epsilon))
				{
					bad_camera = true;
					break;
				}

				depth_delta += abs(depth - uvz.z());// / cos(acos(normal_dot) * 0.5f);
			}

			if (bad_camera)
			{
				continue;
			}

			//if (normal_dot > highest_dot)
			//{
			//	highest_dot = normal_dot;
			//	best_camera = j;
			//}

			if (lowest_depth_delta > depth_delta)
			{
				lowest_depth_delta = depth_delta;
				best_camera = j;
			}
		}

		if (best_camera == -1)
		{
			++withoutUV;
		}
		else
		{
			//Record which camera 'owns' this triangle
			camera_triangles[best_camera].push_back(i);
			mesh->triangle_material_ids_[i] = best_camera;
		}
	}

	std::cout << "There were " << withoutUV << "/" << mesh->triangles_.size() << " triangles without UV coordinates" << std::endl;

	//Not ultimately necessary, but it's nice to be certain
	mesh->triangle_uvs_.clear();
	
	//Used to determine which camera each index belongs to
	std::vector<int> index_claims;
	std::vector<Eigen::Vector3i> new_triangles;

	index_claims.resize(vert_count, -1);
	mesh->triangle_uvs_.resize(vert_count, Eigen::Vector2d(0.0, 0.0));

	//Create new vertices such that we can split relevant UVs by camera
	for (int i = 0; i < camera_count; ++i)
	{
		for (auto triangle_index : camera_triangles[i])
		{
			auto triangle = mesh->triangles_[triangle_index];
			for (int j = 0; j < 3; ++j)
			{
				int vert_loc = index_redirect[triangle(j)];

				if (index_claims[vert_loc] < 0 || index_claims[vert_loc] == i)
				{
					//No other camera has claimed this index, we set appropriate data
					index_claims[vert_loc] = i;

				}
				else
				{
					//If the index is already claimed by a different camera, then duplicate it and redirect the triangles
					index_redirect[triangle(j)] = vert_count;
					vert_loc = index_redirect[triangle(j)];
					++vert_count;

					index_claims.push_back(i);
					mesh->vertices_.push_back(mesh->vertices_[triangle(j)]);
					mesh->vertex_normals_.push_back(mesh->vertex_normals_[triangle(j)]);
					mesh->vertex_colors_.push_back(mesh->vertex_colors_[triangle(j)]);
					mesh->triangle_uvs_.push_back(mesh->triangle_uvs_[triangle(j)]);
				}

				//Ensure that we are pointing to the correct triangle index, if we had to change it previously
				mesh->triangles_[triangle_index](j) = vert_loc;

				//Eliminate the vertex colors
				//mesh->vertex_colors_[vert_loc] = Eigen::Vector3d(0.5,1,0.5);

				//Set UVs
				Eigen::Vector3d uvz = camera_intrinsics[i] *
					(camera_rotations[i] * mesh->vertices_[vert_loc] + camera_positions_original[i]);

				uvz.x() /= (uvz.z() * (double)color_images[i].width_);

				uvz.y() /= (uvz.z() * (double)color_images[i].height_);
				if (useTheBadTexturingMethod)
				{
					uvz.y() += (double)i;
					uvz.y() /= (double)camera_count;
				}
				uvz.y() = 1 - uvz.y();

				//uvz.x() /= uvz.z();
				//uvz.y() /= uvz.z();

				mesh->triangle_uvs_[vert_loc] = Eigen::Vector2d(uvz.x(), uvz.y());
			}
		}
	}

	if (!useTheBadTexturingMethod)
	{
		TextureUnpacker tu;

		auto better_texture = std::make_shared<open3d::geometry::Image>();
		better_texture->width_ = color_images[0].width_;
		better_texture->height_ = color_images[0].height_;
		better_texture->bytes_per_channel_ = color_images[0].bytes_per_channel_;
		better_texture->num_of_channels_ = color_images[0].num_of_channels_;
		better_texture->data_.resize(better_texture->width_ * better_texture->height_ * better_texture->num_of_channels_ * better_texture->bytes_per_channel_);

		ErrorLogger::EXECUTE("Perform UV packing", &tu, &TextureUnpacker::PerformTextureUnpack, &color_images, mesh, &(*better_texture), true);

		return better_texture;
	}
	else
	{
		auto to_return = std::make_shared<open3d::geometry::Image>(open3d::geometry::Image());

		//Combine all the image data into one
		for (int i = 0; i < color_images.size(); ++i)
		{
			to_return->data_.insert(to_return->data_.begin() + to_return->data_.size(), color_images[i].data_.begin(), color_images[i].data_.end());
		}

		to_return->height_ = color_images[0].height_ * color_images.size();
		to_return->width_ = color_images[0].width_;
		to_return->bytes_per_channel_ = color_images[0].bytes_per_channel_;
		to_return->num_of_channels_ = color_images[0].num_of_channels_;

		return to_return;
	}
}

std::shared_ptr<open3d::geometry::Image> MKV_Rendering::CameraManager::CreateUVMapAndTextureAtTimestamp(open3d::geometry::TriangleMesh* mesh, uint64_t timestamp, bool useTheBadTexturingMethod)//, float depth_epsilon)
{
	for (auto cam : camera_data)
	{
		ErrorLogger::EXECUTE("Find Frame At Time " + std::to_string(timestamp), cam, &Abstract_Data::SeekToTime, timestamp);
	}

	return CreateUVMapAndTexture(mesh, useTheBadTexturingMethod);// , depth_epsilon);
}

bool MKV_Rendering::CameraManager::CycleAllCamerasForward()
{
	bool success = true;

	for (auto cam : camera_data)
	{
		success = success && ErrorLogger::EXECUTE("Cycle Camera Forward", cam, &Abstract_Data::CycleCaptureForwards);
	}

	return success;
}

bool MKV_Rendering::CameraManager::CycleAllCamerasBackward()
{
	bool success = true;

	for (auto cam : camera_data)
	{
		success = success && ErrorLogger::EXECUTE("Cycle Camera Backward", cam, &Abstract_Data::CycleCaptureBackwards);
	}

	return success;
}

bool MKV_Rendering::CameraManager::AllCamerasSeekTimestamp(uint64_t timestamp)
{
	bool success = true;

	for (auto cam : camera_data)
	{
		success = success && ErrorLogger::EXECUTE("Camera Seek Time", cam, &Abstract_Data::SeekToTime, timestamp);
	}

	return success;
}

open3d::t::geometry::TriangleMesh MKV_Rendering::CameraManager::GetMeshAtTimestamp(VoxelGridData* data, uint64_t timestamp)
{
	return ErrorLogger::EXECUTE("Construct Voxel Grid", this, &MKV_Rendering::CameraManager::GetVoxelGridAtTimestamp, data, timestamp).ExtractSurfaceMesh(0.0f);
}

open3d::t::geometry::TSDFVoxelGrid MKV_Rendering::CameraManager::GetVoxelGridAtTimestamp(VoxelGridData* data, uint64_t timestamp)
{
	open3d::core::Device device(data->device_code);

	open3d::t::geometry::TSDFVoxelGrid voxel_grid(

		{
			{"tsdf", open3d::core::Dtype::Float32},
			{"weight", open3d::core::Dtype::UInt16},
			{"color", open3d::core::Dtype::UInt16}
		},

		data->voxel_size, data->signed_distance_field_truncation,
		16, data->blocks, device
	);

	for (auto cam : camera_data)
	{
		ErrorLogger::EXECUTE("Find Frame At Time " + std::to_string(timestamp), cam, &Abstract_Data::SeekToTime, timestamp);

		ErrorLogger::EXECUTE("Pack Frame into Voxel Grid", cam, &Abstract_Data::PackIntoVoxelGrid, &voxel_grid, data);
	}

	return voxel_grid;
}

std::vector<open3d::geometry::RGBDImage> MKV_Rendering::CameraManager::ExtractImageVectorAtTimestamp(uint64_t timestamp)
{
	std::vector<open3d::geometry::RGBDImage> to_return;

	for (auto cam : camera_data)
	{
		ErrorLogger::EXECUTE("Find Frame At Time " + std::to_string(timestamp), cam, &Abstract_Data::SeekToTime, timestamp);

		to_return.push_back(*ErrorLogger::EXECUTE("Extract RGBD Image Vector", cam, &Abstract_Data::GetFrameRGBD));
	}

	return to_return;
}

void MKV_Rendering::CameraManager::GetTrajectories(open3d::camera::PinholeCameraTrajectory &traj)
{
	for (auto cam : camera_data)
	{
		open3d::camera::PinholeCameraParameters params;

		traj.parameters_.push_back(
			cam->GetParameters()
		);
	}
}

void MKV_Rendering::CameraManager::CreateSSMVFolder(
	VoxelGridData *vgd, std::string destination_folder, uint64_t timestamp, std::string camera_calib_filename, std::string image_list_filename, std::string image_base_name, std::string mesh_name
)
{
	std::ofstream camera_data_file;
	std::ofstream image_data_file;

	std::filesystem::create_directories(destination_folder);

	camera_data_file.open(destination_folder + "/" + camera_calib_filename);
	image_data_file.open(destination_folder + "/" + image_list_filename);

	camera_data_file << camera_data.size() << std::endl;

	for (int i = 0; i < camera_data.size(); ++i)
	{
		auto int_mat = camera_data[i]->GetIntrinsicMat();
		auto ext_mat = camera_data[i]->GetExtrinsicMat();

		for (int j = 0; j < 3; ++j)
		{
			for (int k = 0; k < 3; ++k)
			{
				camera_data_file << int_mat(j, k) << " ";
			}
		}

		for (int j = 0; j < 3; ++j)
		{
			for (int k = 0; k < 3; ++k)
			{
				camera_data_file << ext_mat(j, k) << " ";
			}
		}

		for (int j = 0; j < 3; ++j)
		{
			camera_data_file << ext_mat(j, 3) << " ";
		}

		camera_data[i]->SeekToTime(timestamp);
		auto rgbd_image = camera_data[i]->GetFrameRGBD();

		camera_data_file << rgbd_image->color_.width_ << " ";
		camera_data_file << rgbd_image->color_.height_ << std::endl;



		std::string image_name = GetNumberFixedLength(i, 8) + image_base_name;
		std::string image_address = destination_folder + "/" + image_name;

		open3d::io::WriteImageToPNG(image_address, rgbd_image->color_);

		image_data_file << image_address << std::endl;
	}

	auto mesh = GetMeshAtTimestamp(vgd, timestamp).ToLegacyTriangleMesh();

	open3d::io::WriteTriangleMesh(destination_folder + "/" + mesh_name, mesh);

	image_data_file.close();
	camera_data_file.close();
}

uint64_t MKV_Rendering::CameraManager::GetHighestTimestamp()
{
	uint64_t to_return = 0;
	
	for (auto cam : camera_data)
	{
		auto timestamp = cam->GetTimestampCached();

		if (to_return < timestamp)
		{
			to_return = timestamp;
		}
	}

	return to_return;
}

void MKV_Rendering::CameraManager::MakeAnErrorOnPurpose(bool cause_abort)
{
	CauseError(cause_abort);
}
