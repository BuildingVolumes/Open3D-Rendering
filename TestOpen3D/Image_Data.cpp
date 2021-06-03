#include "Image_Data.h"

#include <fstream>

#include "AdditionalUtilities.h"

void MKV_Rendering::Image_Data::LoadImages()
{
    open3d::utility::filesystem::ListFilesInDirectory(color_folder, color_files);
    open3d::utility::filesystem::ListFilesInDirectory(depth_folder, depth_files);

    if (color_files.size() == 0)
    {
        ErrorLogger::LOG_ERROR("No color files present!", true);
    }

    if (depth_files.size() == 0)
    {
        ErrorLogger::LOG_ERROR("No color files present!", true);
    }

    std::sort(color_files.begin(), color_files.end());
    std::sort(depth_files.begin(), depth_files.end());

    //Shorter than an if statement, and supposedly faster
    int color_greater = (color_files.size() > depth_files.size());
    int max_files = color_greater * color_files.size() + (1 - color_greater) * depth_files.size();

    color_files.resize(max_files);
    depth_files.resize(max_files);
}

void MKV_Rendering::Image_Data::GetIntrinsicTensor()
{

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

    auto params = calibration.depth_camera_calibration.intrinsics.parameters;

    intrinsic_t = open3d::core::Tensor::Init<double>({
            {params.param.fx, 0, params.param.cx},
            {0, params.param.fy, params.param.cy},
            {0, 0, 1}
        });
}

void MKV_Rendering::Image_Data::GetExtrinsicTensor()
{
    if (calibration_file == "")
    {
        Eigen::Matrix4d matId = Eigen::Matrix4d::Identity();
        extrinsic_t = open3d::core::eigen_converter::EigenMatrixToTensor(matId);
        return;
    }

    std::string file_contents = "";
    std::ifstream calib_file = std::ifstream(calibration_file);

    while (!calib_file.eof())
    {
        std::string single_content = "";
        std::getline(calib_file, single_content);
        single_content += " ";

        file_contents.append(single_content);
    }

    Eigen::Vector3d translation;
    Eigen::Matrix3d r_mat_3 = Eigen::Matrix3d::Identity();

    std::vector<std::string> extrinsic_data;
    SplitString(file_contents, extrinsic_data, ' ');

    for (int j = 0; j < 3; ++j)
    {
        translation[j] = std::stof(extrinsic_data[j]);
    }

    for (int j = 0; j < 9; ++j)
    {
        r_mat_3(j / 3, j % 3) = std::stof(extrinsic_data[j + 3]);
    }

    Eigen::Matrix4d final_mat = Eigen::Matrix4d::Identity();

    final_mat.block<3, 3>(0, 0) = r_mat_3;
    final_mat.block<3, 1>(0, 3) = r_mat_3 * translation;

    Eigen::Matrix4d final_mat2 = final_mat.inverse();

    extrinsic_t = open3d::core::eigen_converter::EigenMatrixToTensor(final_mat2);
}

open3d::geometry::Image MKV_Rendering::Image_Data::TransformDepth(open3d::geometry::Image *old_depth, open3d::geometry::Image* color)
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

    k4a_image_create_from_buffer(
        K4A_IMAGE_FORMAT_DEPTH16, width, height,
        width * sizeof(uint16_t), new_depth.data_.data(),
        width * height * sizeof(uint16_t), NULL, NULL,
        &k4a_transformed_depth);
    if (K4A_RESULT_SUCCEEDED !=
        k4a_transformation_depth_image_to_color_camera(
            transform, k4a_depth, k4a_transformed_depth)) {
        ErrorLogger::LOG_ERROR("Failed to transform depth frame to color frame at " + std::to_string(_timestamp) + ".", true);
    }

    k4a_image_release(k4a_depth);
    k4a_image_release(k4a_transformed_depth);

    return new_depth;
}

MKV_Rendering::Image_Data::Image_Data(std::string root_folder, std::string color_folder, std::string depth_folder, std::string intrinsics, std::string extrinsics, double FPS) : Abstract_Data(root_folder)
{
    this->color_folder = root_folder + "/" + color_folder;
    this->depth_folder = root_folder + "/" + depth_folder;
    this->intrinsics_file = root_folder + "/" + intrinsics + ".json";
    this->calibration_file = root_folder + "/" + extrinsics + ".log";
    this->FPS = FPS;
    current_frame = 0;

    ErrorLogger::EXECUTE("Load Images", this, &Image_Data::LoadImages);

    ErrorLogger::EXECUTE("Create Intrinsic Tensor", this, &Image_Data::GetIntrinsicTensor);
    ErrorLogger::EXECUTE("Create Extrinsic Tensor", this, &Image_Data::GetExtrinsicTensor);

    transform = k4a_transformation_create(&calibration);
}

MKV_Rendering::Image_Data::~Image_Data()
{
}

uint64_t MKV_Rendering::Image_Data::GetCaptureTimestamp()
{
    uint64_t to_return = (current_frame * 1000000.0 / FPS);


    return (current_frame * 1000000.0 / FPS);
}

bool MKV_Rendering::Image_Data::CycleCaptureForwards()
{
    ++current_frame;

    if (current_frame >= color_files.size())
    {
        current_frame = color_files.size() - 1;
        ErrorLogger::LOG_ERROR("Reached end of image folder!");
        return false;
    }

    return true;
}

bool MKV_Rendering::Image_Data::CycleCaptureBackwards()
{
    --current_frame;

    if (current_frame < 0)
    {
        current_frame = 0;
        ErrorLogger::LOG_ERROR("Reached beginning of image folder!");
        return false;
    }

    return true;
}

bool MKV_Rendering::Image_Data::SeekToTime(uint64_t time)
{
    current_frame = (time * FPS / 1000000.0);

    std::cout << "seeking to " << current_frame << std::endl;

    if (current_frame >= color_files.size())
    {
        current_frame = color_files.size() - 1;
        ErrorLogger::LOG_ERROR("Reached end of image folder!");
        return false;
    }

    if (current_frame < 0)
    {
        current_frame = 0;
        ErrorLogger::LOG_ERROR("Reached beginning of image folder!");
        return false;
    }

    return true;
}

void MKV_Rendering::Image_Data::PackIntoVoxelGrid(open3d::t::geometry::TSDFVoxelGrid* grid, VoxelGridData* data)
{
    auto color = (*open3d::t::io::CreateImageFromFile(color_files[current_frame]));
    auto depth = (*open3d::t::io::CreateImageFromFile(depth_files[current_frame]));// .ToLegacyImage();

    //auto new_depth = open3d::t::geometry::Image::FromLegacyImage(ErrorLogger::EXECUTE("Transforming Depth", this, &Image_Data::TransformDepth, &depth, &color.ToLegacyImage()));

    std::cout << intrinsic_t.ToString() << std::endl;
    std::cout << extrinsic_t.ToString() << std::endl;

    color.To(grid->GetDevice());
    depth.To(grid->GetDevice());

    grid->Integrate(depth, color,
        intrinsic_t, extrinsic_t,
        data->depth_scale, data->depth_max);
}
