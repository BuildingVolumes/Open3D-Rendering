#include "Livescan_Data.h"

#include <fstream>

#include "AdditionalUtilities.h"

void MKV_Rendering::Livescan_Data::UpdateTimestamp()
{
	_timestamp = (double)current_frame / FPS * 1000000.0;
}

void MKV_Rendering::Livescan_Data::LoadImages()
{
	std::vector<std::string> all_files;

	open3d::utility::filesystem::ListFilesInDirectory(folder_name, all_files);

	for (auto s : all_files)
	{
		std::vector<std::string> split_extension;
		SplitString(s, split_extension, '.');

		if (split_extension.back() == "jpg")
		{
			std::vector<std::string> split_num;
			SplitString(split_extension.front(), split_num, '_');
			int loc = std::stoi(split_num.back());
			color_files[loc] = s;
		}
		else if (split_extension.back() == "png")
		{
			std::vector<std::string> split_num;
			SplitString(split_extension.front(), split_num, '_');
			int loc = std::stoi(split_num.back());
			depth_files[loc] = s;
		}
		else if (split_extension.back() == "json")
		{
			std::cout << s << std::endl;
			intrinsics_file = s;
		}
	}
}

void MKV_Rendering::Livescan_Data::GetIntrinsicTensor()
{
	double ratio = 720.0 / 1080.0;

	std::ifstream intrinsic_file = std::ifstream(intrinsics_file);
	std::string file_contents;

	intrinsic_file.seekg(0, std::ios::end);
	size_t file_length = intrinsic_file.tellg();
	intrinsic_file.seekg(0, std::ios::beg);

	file_contents.resize(file_length);
	intrinsic_file.read(&file_contents[0], file_length);

	intrinsic_file.close();

	k4a_result_t res = k4a_calibration_get_from_raw(&file_contents[0], file_contents.size() + 1, k4a_depth_mode_t::K4A_DEPTH_MODE_NFOV_UNBINNED, k4a_color_resolution_t::K4A_COLOR_RESOLUTION_1080P, &calibration);
	if (res == k4a_result_t::K4A_RESULT_FAILED)
	{
		std::cout << "ERROR: FAILED TO READ INTRINSICS JSON!" << std::endl;
	}

	calibration.color_camera_calibration.resolution_width *= ratio;
	calibration.color_camera_calibration.resolution_height *= ratio;
	calibration.color_camera_calibration.intrinsics.parameters.param.fx *= ratio;
	calibration.color_camera_calibration.intrinsics.parameters.param.fy *= ratio;
	calibration.color_camera_calibration.intrinsics.parameters.param.cx *= ratio;
	calibration.color_camera_calibration.intrinsics.parameters.param.cy *= ratio;

	std::cout << calibration.color_camera_calibration.resolution_width << ", " << calibration.color_camera_calibration.resolution_height << std::endl;

	std::cout << calibration.depth_camera_calibration.resolution_width << ", " << calibration.depth_camera_calibration.resolution_height << std::endl;

	auto params = calibration.color_camera_calibration.intrinsics.parameters;

	calibration.color_resolution = K4A_COLOR_RESOLUTION_720P;

	intrinsic_mat = Eigen::Matrix3d::Identity();
	intrinsic_mat(0, 0) = params.param.fx;
	intrinsic_mat(1, 1) = params.param.fy;
	intrinsic_mat(0, 2) = params.param.cx;
	intrinsic_mat(1, 2) = params.param.cy;

	intrinsic_t = open3d::core::Tensor::Init<double>({
			{params.param.fx, 0, params.param.cx},
			{0, params.param.fy, params.param.cy},
			{0, 0, 1}
		});
}

void MKV_Rendering::Livescan_Data::GetExtrinsicTensor()
{
	for (int i = 0; i < extrinsic_individual.size(); ++i)
	{
		extrinsic_mat.block<1, 1>(i / 4, i % 4)[0] = std::stof(extrinsic_individual[i]);
	}

	extrinsic_mat = extrinsic_mat.inverse();

	extrinsic_t = open3d::core::eigen_converter::EigenMatrixToTensor(extrinsic_mat);
}

open3d::geometry::Image MKV_Rendering::Livescan_Data::TransformDepth(open3d::geometry::Image* old_depth, open3d::geometry::Image* color)
{
	open3d::geometry::Image new_depth;

	auto width = color->width_;
	auto height = color->height_;
	auto stride = color->BytesPerLine();

	new_depth.height_ = height;
	new_depth.width_ = width;
	new_depth.bytes_per_channel_ = old_depth->bytes_per_channel_;
	new_depth.num_of_channels_ = old_depth->num_of_channels_;
	new_depth.data_.resize(height * width * new_depth.bytes_per_channel_);

	k4a_image_t k4a_transformed_depth = nullptr;
	k4a_image_t k4a_depth = nullptr;

	if (K4A_RESULT_SUCCEEDED !=
		k4a_image_create_from_buffer(k4a_image_format_t::K4A_IMAGE_FORMAT_DEPTH16, old_depth->width_, old_depth->height_, old_depth->BytesPerLine(),
			old_depth->data_.data(), old_depth->data_.size(), nullptr, nullptr, &k4a_depth))
	{
		ErrorLogger::LOG_ERROR("Failed to create a source depth image at " + std::to_string(_timestamp) + ".", true);
	}

	if (K4A_RESULT_SUCCEEDED !=
		k4a_image_create_from_buffer(k4a_image_format_t::K4A_IMAGE_FORMAT_DEPTH16, new_depth.width_, new_depth.height_, new_depth.BytesPerLine(),
			new_depth.data_.data(), new_depth.data_.size(), nullptr, nullptr, &k4a_transformed_depth))
	{
		ErrorLogger::LOG_ERROR("Failed to create a destination depth image at " + std::to_string(_timestamp) + ".", true);
	}

	if (K4A_RESULT_SUCCEEDED !=
		k4a_transformation_depth_image_to_color_camera(
			transform, k4a_depth, k4a_transformed_depth)) {
		ErrorLogger::LOG_ERROR("Failed to transform depth frame to color frame at " + std::to_string(_timestamp) + ".", true);
	}

	k4a_image_release(k4a_depth);
	k4a_image_release(k4a_transformed_depth);

	return new_depth;
}

MKV_Rendering::Livescan_Data::Livescan_Data(std::string data_folder, std::vector<std::string>& extrinsics, double FPS) : Abstract_Data(data_folder), FPS(FPS)
{
	for (auto s : extrinsics)
	{
		std::vector<std::string> split;

		SplitString(s, split, '\t');

		for (auto s_2 : split)
		{
			extrinsic_individual.push_back(s_2);
		}
	}

	ErrorLogger::EXECUTE("Load Images", this, &Livescan_Data::LoadImages);

	ErrorLogger::EXECUTE("Create Intrinsic Tensor", this, &Livescan_Data::GetIntrinsicTensor);
	ErrorLogger::EXECUTE("Create Extrinsic Tensor", this, &Livescan_Data::GetExtrinsicTensor);


	auto im = open3d::t::io::CreateImageFromFile(color_files[current_frame]);

	int imageWidth = im->GetCols();
	int imageHeight = im->GetRows();

	transform = k4a_transformation_create(&calibration);
}

MKV_Rendering::Livescan_Data::~Livescan_Data()
{
}

uint64_t MKV_Rendering::Livescan_Data::GetCaptureTimestamp()
{
	return _timestamp;
}

bool MKV_Rendering::Livescan_Data::CycleCaptureForwards()
{
	++current_frame;

	if (current_frame >= color_files.size())
	{
		current_frame = color_files.size() - 1;
		UpdateTimestamp();
		ErrorLogger::LOG_ERROR("Reached end of image folder!");
		return false;
	}

	UpdateTimestamp();
	return true;
}

bool MKV_Rendering::Livescan_Data::CycleCaptureBackwards()
{
	--current_frame;

	if (current_frame < 0)
	{
		current_frame = 0;
		UpdateTimestamp();
		ErrorLogger::LOG_ERROR("Reached beginning of image folder!");
		return false;
	}

	UpdateTimestamp();
	return true;
}

bool MKV_Rendering::Livescan_Data::SeekToTime(uint64_t time)
{
	auto time_seconds = time / 1000000.0;

	current_frame = (time_seconds * FPS);

	std::cout << "seeking to " << current_frame << std::endl;

	auto upper = color_files.upper_bound(current_frame);

	if (upper == color_files.end())
	{
		current_frame = color_files.rbegin()->first;
		UpdateTimestamp();
		ErrorLogger::LOG_ERROR("Reached end of image folder!");
		return false;
	}

	UpdateTimestamp();
	std::cout << "Frame is at " << _timestamp << "\n" << std::endl;

	return true;
}

std::shared_ptr<open3d::geometry::RGBDImage> MKV_Rendering::Livescan_Data::GetFrameRGBD()
{
	auto col = (*open3d::t::io::CreateImageFromFile(color_files.lower_bound(current_frame)->second)).ToLegacyImage();
	auto dep = (*open3d::t::io::CreateImageFromFile(depth_files.lower_bound(current_frame)->second)).ToLegacyImage();

	auto new_dep = ErrorLogger::EXECUTE("Transforming Depth", this, &Livescan_Data::TransformDepth, &dep, &col);

	std::cout << color_files[current_frame] << std::endl;

	return std::make_shared<open3d::geometry::RGBDImage>(
		col, new_dep
		);
}

open3d::camera::PinholeCameraParameters MKV_Rendering::Livescan_Data::GetParameters()
{
	open3d::camera::PinholeCameraParameters to_return;

	to_return.extrinsic_ = extrinsic_mat;

	to_return.intrinsic_ = open3d::camera::PinholeCameraIntrinsic(
		calibration.color_camera_calibration.resolution_width,
		calibration.color_camera_calibration.resolution_height,
		calibration.color_camera_calibration.intrinsics.parameters.param.fx,
		calibration.color_camera_calibration.intrinsics.parameters.param.fy,
		calibration.color_camera_calibration.intrinsics.parameters.param.cx,
		calibration.color_camera_calibration.intrinsics.parameters.param.cy
	);

	return to_return;
}

void MKV_Rendering::Livescan_Data::PackIntoVoxelGrid(open3d::t::geometry::TSDFVoxelGrid* grid, VoxelGridData* data)
{
	auto color = (*open3d::t::io::CreateImageFromFile(color_files.lower_bound(current_frame)->second));
	auto depth = (*open3d::t::io::CreateImageFromFile(depth_files.lower_bound(current_frame)->second)).ToLegacyImage();

	std::cout << "current frame: " << current_frame << "," << color_files[current_frame] << std::endl;

	auto new_depth = open3d::t::geometry::Image::FromLegacyImage(
		ErrorLogger::EXECUTE("Transforming Depth", this, &Livescan_Data::TransformDepth, &depth, &color.ToLegacyImage())
	);

	color.To(grid->GetDevice());
	new_depth.To(grid->GetDevice());

	grid->Integrate(new_depth, color,
		intrinsic_t, extrinsic_t,
		data->depth_scale, data->depth_max);
}

void MKV_Rendering::Livescan_Data::PackIntoOldVoxelGrid(open3d::geometry::VoxelGrid* grid)
{
	auto rgbd = GetFrameRGBD();

	auto params = GetParameters();

	grid->CarveSilhouette(rgbd->depth_, params, true);
}
