#include "Image_Data.h"

#include <fstream>

#include "AdditionalUtilities.h"

void MKV_Rendering::Image_Data::UpdateTimestamp()
{
    if (FPS > 0)
    {
        _timestamp = current_frame * 1000000.0 / FPS;
    }
}

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
        ErrorLogger::LOG_ERROR("No depth files present!", true);
    }

    std::sort(color_files.begin(), color_files.end());
    std::sort(depth_files.begin(), depth_files.end());

    //Shorter than an if statement, and supposedly faster
    int color_greater = (color_files.size() > depth_files.size());
    int max_files = color_greater * color_files.size() + (1 - color_greater) * depth_files.size();

    color_files.resize(max_files);
    depth_files.resize(max_files);

    if (FPS <= 0)
    {
        std::cout << "No FPS attribute present, parsing images for timestamps..." << std::endl;

        try
        {
            for (int i = 0; i < max_files; ++i)
            {
                std::vector<std::string> split_file;
                std::vector<std::string> split_extension;
                SplitString(color_files[i], split_file, '_');
                SplitString(split_file.back(), split_extension, '.');

                auto timestamp = std::stoull(split_extension[0]);
                color_timestamps[timestamp] = color_files[i];
                depth_timestamps[timestamp] = color_files[i];
            }
        }
        catch (const std::exception& e)
        {
            ErrorLogger::LOG_ERROR("Error when parsing timestamps: " + std::string(e.what()), true);
        }
    }
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

    auto params = calibration.color_camera_calibration.intrinsics.parameters;

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

void MKV_Rendering::Image_Data::GetExtrinsicTensor()
{
    if (calibration_file == "")
    {
        extrinsic_mat = Eigen::Matrix4d::Identity();
        extrinsic_t = open3d::core::eigen_converter::EigenMatrixToTensor(extrinsic_mat);
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

    extrinsic_mat = final_mat.inverse();

    extrinsic_t = open3d::core::eigen_converter::EigenMatrixToTensor(extrinsic_mat);
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

MKV_Rendering::Image_Data::Image_Data(std::string root_folder, std::string color_folder, std::string depth_folder, std::string intrinsics, std::string extrinsics, std::string FPS) : Abstract_Data(root_folder)
{
    this->color_folder = root_folder + "/" + color_folder;
    this->depth_folder = root_folder + "/" + depth_folder;
    this->intrinsics_file = root_folder + "/" + intrinsics + ".json";
    this->calibration_file = root_folder + "/" + extrinsics + ".log";

    if (FPS != "")
    {
        this->FPS = std::stod(FPS);
        _timestamp = 0;
    }
    else
    {
        this->FPS = 0;
    }

    current_frame = 0;

    ErrorLogger::EXECUTE("Load Images", this, &Image_Data::LoadImages);

    ErrorLogger::EXECUTE("Create Intrinsic Tensor", this, &Image_Data::GetIntrinsicTensor);
    ErrorLogger::EXECUTE("Create Extrinsic Tensor", this, &Image_Data::GetExtrinsicTensor);

    auto im = open3d::t::io::CreateImageFromFile(color_files[current_frame]);

    int imageWidth = im->GetCols();
    int imageHeight = im->GetRows();

    transform = k4a_transformation_create(&calibration);
}

MKV_Rendering::Image_Data::~Image_Data()
{
}

uint64_t MKV_Rendering::Image_Data::GetCaptureTimestamp()
{
    return _timestamp;
}

bool MKV_Rendering::Image_Data::CycleCaptureForwards()
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

bool MKV_Rendering::Image_Data::CycleCaptureBackwards()
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

bool MKV_Rendering::Image_Data::SeekToTime(uint64_t time)
{
    auto time_seconds = time / 1000000.0;

    if (FPS > 0)
    {
        current_frame = (time_seconds * FPS);
    }
    else
    {
        auto p = color_timestamps.lower_bound(time);

        if (p == color_timestamps.begin())
        {
            current_frame = 0;
            ErrorLogger::LOG_ERROR("Time stamp out of bounds!");
            return false;
        }
        else
        {
            auto it = std::find(color_files.begin(), color_files.end(), p->second);

            current_frame = it - color_files.begin();

            _timestamp = p->first;
        }
    }

    std::cout << "seeking to " << current_frame << std::endl;

    if (current_frame >= color_files.size())
    {
        current_frame = color_files.size() - 1;
        UpdateTimestamp();
        ErrorLogger::LOG_ERROR("Reached end of image folder!");
        return false;
    }

    if (current_frame < 0)
    {
        current_frame = 0;
        UpdateTimestamp();
        ErrorLogger::LOG_ERROR("Reached beginning of image folder!");
        return false;
    }

    UpdateTimestamp();

    std::cout << "Frame is at " << _timestamp << "\n" << std::endl;

    return true;
}

std::shared_ptr<open3d::geometry::RGBDImage> MKV_Rendering::Image_Data::GetFrameRGBD()
{
    auto col = (*open3d::t::io::CreateImageFromFile(color_files[current_frame])).ToLegacyImage();
    auto dep = (*open3d::t::io::CreateImageFromFile(depth_files[current_frame])).ToLegacyImage();

    std::cout << color_files[current_frame] << std::endl;

    return std::make_shared<open3d::geometry::RGBDImage>(
        col, dep
    );
}

open3d::camera::PinholeCameraParameters MKV_Rendering::Image_Data::GetParameters()
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

void MKV_Rendering::Image_Data::PackIntoVoxelGrid(open3d::t::geometry::TSDFVoxelGrid* grid, VoxelGridData* data)
{
    auto color = (*open3d::t::io::CreateImageFromFile(color_files[current_frame]));
    auto depth = (*open3d::t::io::CreateImageFromFile(depth_files[current_frame]));// .ToLegacyImage();

    //auto new_depth = open3d::t::geometry::Image::FromLegacyImage(ErrorLogger::EXECUTE("Transforming Depth", this, &Image_Data::TransformDepth, &depth, &color.ToLegacyImage()));

    //std::cout << intrinsic_t.ToString() << std::endl;
    //std::cout << extrinsic_t.ToString() << std::endl;

    color.To(grid->GetDevice());
    depth.To(grid->GetDevice());

    grid->Integrate(depth, color,
        intrinsic_t, extrinsic_t,
        data->depth_scale, data->depth_max);
}

void MKV_Rendering::Image_Data::PackIntoOldVoxelGrid(open3d::geometry::VoxelGrid* grid)
{
    auto rgbd = GetFrameRGBD();

    auto params = GetParameters();

    grid->CarveSilhouette(rgbd->depth_, params, true);
}