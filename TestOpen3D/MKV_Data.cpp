#include "MKV_Data.h"
#include "open3d/io/sensor/azure_kinect/MKVMetadata.h"
#include "open3d/io/sensor/azure_kinect/K4aPlugin.h"
#include "ErrorLogger.h"

#include "AdditionalUtilities.h"

#include <fstream>
#include <turbojpeg.h>

using namespace MKV_Rendering;

MKV_Data* MKV_Data::main_camera_data = nullptr;

void MKV_Rendering::MKV_Data::Initialize(std::string my_folder, std::string mkv_name, std::string calibration_name)
{
    if (mkv_name != "")
        mkv_file = my_folder + "/" + mkv_name + ".mkv";
    else
        mkv_file = "";

    if (calibration_name != "")
        calibration_file = my_folder + "/" + calibration_name + ".log";
    else
        calibration_file = "";

    std::vector<std::string> files;
    open3d::utility::filesystem::ListFilesInDirectory(my_folder, files);

    for (auto _file : files)
    {
        std::vector<std::string> filename_and_extension;

        SplitString(_file, filename_and_extension, '.');

        if (filename_and_extension.back() == "mkv" && mkv_name == "")
        {
            if (mkv_file == "")
                mkv_file = _file;
            else
                ErrorLogger::LOG_ERROR("Multiple MKVs in the same folder, " + my_folder + "!");
        }
        else if (filename_and_extension.back() == "log" && calibration_name == "")
        {
            if (calibration_file == "")
                calibration_file = _file;
            else
                ErrorLogger::LOG_ERROR("Multiple calibration files in the same folder, " + my_folder + "!");
        }
    }

    if (mkv_file == "")
        ErrorLogger::LOG_ERROR("No MKV present!", true);

}

void MKV_Data::Calibrate()
{
    if (k4a_result_t::K4A_RESULT_SUCCEEDED !=
        k4a_playback_open(mkv_file.c_str(), &handle))
    {
        ErrorLogger::LOG_ERROR("Failed to open file: " + mkv_file, true);
    }

    if (k4a_result_t::K4A_RESULT_SUCCEEDED !=
        k4a_playback_get_record_configuration(handle, &record_config))
    {
        ErrorLogger::LOG_ERROR("Failed to get record configuration from: " + mkv_file, true);
    }

    switch (record_config.wired_sync_mode)
    {
    case K4A_WIRED_SYNC_MODE_MASTER:
        if (main_camera_data == nullptr)
        {
            main_camera_data = this;
            std::cout << mkv_file << " set as main camera" << std::endl;
        }
        else
        {
            ErrorLogger::LOG_ERROR("Conflict between " + main_camera_data->mkv_file + " and " + mkv_file + " over main camera", true);
        }
        break;
    case K4A_WIRED_SYNC_MODE_SUBORDINATE:
        std::cout << mkv_file << " set as subordinate camera" << std::endl;
        break;
    default:
        ErrorLogger::LOG_ERROR("Bad record configuration on: " + mkv_file, true);
    }
    
    start_offset = record_config.start_timestamp_offset_usec;

    std::cout << "Start Offset: " << start_offset << std::endl;
}

void MKV_Data::GetPlaybackDataRaw()
{
    if (!playback_data.empty())
    {
        playback_data.clear();
    }

    size_t data_len = 0;

    k4a_playback_get_raw_calibration(handle, nullptr, &data_len);

    playback_data.resize(data_len);

    std::cout << "Data size: " << data_len << std::endl;

    switch (k4a_playback_get_raw_calibration(handle, &playback_data[0], &data_len))
    {
    case k4a_buffer_result_t::K4A_BUFFER_RESULT_TOO_SMALL:
        ErrorLogger::LOG_ERROR("Buffer was to small in: " + mkv_file, true);
        break;
    case k4a_buffer_result_t::K4A_BUFFER_RESULT_FAILED:
        ErrorLogger::LOG_ERROR("Failed to make playback calibration: " + mkv_file, true);
        break;
    default:
        break;
    }

    if (k4a_result_t::K4A_RESULT_SUCCEEDED !=
        k4a_calibration_get_from_raw(
            (char*)playback_data.data(), playback_data.size() + 1,
            k4a_depth_mode_t::K4A_DEPTH_MODE_NFOV_UNBINNED,
            k4a_color_resolution_t::K4A_COLOR_RESOLUTION_1080P, &calibration
        ))
    {
        ErrorLogger::LOG_ERROR("Failed to turn raw data into calibration: " + mkv_file, true);
    }
}

void MKV_Data::GetIntrinsicTensor()
{
    auto params = calibration.color_camera_calibration.intrinsics.parameters.param;

    auto focalX = params.fx;
    auto focalY = params.fy;
    auto principleX = params.cx;
    auto principleY = params.cy;

    intrinsic_mat = Eigen::Matrix3d::Identity();
    intrinsic_mat(0, 0) = params.fx;
    intrinsic_mat(1, 1) = params.fy;
    intrinsic_mat(0, 2) = params.cx;
    intrinsic_mat(1, 2) = params.cy;

    intrinsic_t = open3d::core::Tensor::Init<double>({
        {params.fx, 0, params.cx},
        {0, params.fy, params.cy},
        {0, 0, 1}
        });

    std::cout << intrinsic_t.ToString() << std::endl;

    std::cout <<
        calibration.depth_camera_calibration.intrinsics.parameters.param.fx << ", " <<
        calibration.depth_camera_calibration.intrinsics.parameters.param.fy << ", " <<
        calibration.depth_camera_calibration.intrinsics.parameters.param.cx << ", " <<
        calibration.depth_camera_calibration.intrinsics.parameters.param.cy << std::endl;
}

void MKV_Data::GetExtrinsicTensor()
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

void MKV_Data::ConvertBGRAToRGB(open3d::geometry::Image& bgra, open3d::geometry::Image& rgb)
{
    if (bgra.bytes_per_channel_ != 1) {
        ErrorLogger::LOG_ERROR("BGRA input image must have 1 byte per channel.", true);
    }
    if (rgb.bytes_per_channel_ != 1) {
        ErrorLogger::LOG_ERROR("RGB output image must have 1 byte per channel.", true);
    }
    if (bgra.num_of_channels_ != 4) {
        ErrorLogger::LOG_ERROR("BGRA input image must have 4 channels.", true);
    }
    if (rgb.num_of_channels_ != 3) {
        ErrorLogger::LOG_ERROR("RGB output image must have 3 channels.", true);
    }
    if (bgra.width_ != rgb.width_ || bgra.height_ != rgb.height_) {
        ErrorLogger::LOG_ERROR(
            "BGRA input image and RGB output image have different dimensions.", 
            true);
    }

#ifdef _WIN32
#pragma omp parallel for schedule(static)
#else
#pragma omp parallel for collapse(3) schedule(static)
#endif
    for (int v = 0; v < bgra.height_; ++v) {
        for (int u = 0; u < bgra.width_; ++u) {
            for (int c = 0; c < 3; ++c) {
                *rgb.PointerAt<uint8_t>(u, v, c) =
                    *bgra.PointerAt<uint8_t>(u, v, 2 - c);
            }
        }
    }
}

std::shared_ptr<open3d::geometry::RGBDImage> MKV_Data::DecompressCapture()
{
    static std::shared_ptr<open3d::geometry::Image> color_buffer = nullptr;
    static std::shared_ptr<open3d::geometry::RGBDImage> rgbd_buffer = nullptr;

    if (color_buffer == nullptr) {
        color_buffer = std::make_shared<open3d::geometry::Image>();
    }
    if (rgbd_buffer == nullptr) {
        rgbd_buffer = std::make_shared<open3d::geometry::RGBDImage>();
    }

    k4a_image_t k4a_color = k4a_capture_get_color_image(*capture);
    k4a_image_t k4a_depth = k4a_capture_get_depth_image(*capture);
    if (k4a_color == nullptr || k4a_depth == nullptr) {
        ErrorLogger::LOG_ERROR("Capture at " + std::to_string(_timestamp) + " empty, skipping");
        return nullptr;
    }

    /* Process color */
    if (K4A_IMAGE_FORMAT_COLOR_MJPG !=
        k4a_image_get_format(k4a_color)) {
        ErrorLogger::LOG_ERROR("Unexpected image format at " + std::to_string(_timestamp) + ". The stream may have been corrupted.");
        return nullptr;
    }

    int width = k4a_image_get_width_pixels(k4a_color);
    int height = k4a_image_get_height_pixels(k4a_color);

    imageWidth = width;
    imageHeight = height;

    /* resize */
    rgbd_buffer->color_.Prepare(width, height, 3, sizeof(uint8_t));
    color_buffer->Prepare(width, height, 4, sizeof(uint8_t));

    tjhandle tjHandle;
    tjHandle = tjInitDecompress();
    if (0 !=
        tjDecompress2(tjHandle, k4a_image_get_buffer(k4a_color),
            static_cast<unsigned long>(
                k4a_image_get_size(k4a_color)),
            color_buffer->data_.data(), width, 0 /* pitch */, height,
            TJPF_BGRA, TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE)) {
        ErrorLogger::LOG_ERROR("Failed to decompress color image at " + std::to_string(_timestamp) + ".");
        return nullptr;
    }
    tjDestroy(tjHandle);
    ErrorLogger::EXECUTE(
        "Converting Image Type from BGRA to RGB", 
        this, 
        &MKV_Data::ConvertBGRAToRGB, *color_buffer, rgbd_buffer->color_);

    /* transform depth to color plane */
    k4a_image_t k4a_transformed_depth = nullptr;
    if (transform) {
        rgbd_buffer->depth_.Prepare(width, height, 1, sizeof(uint16_t));
        k4a_image_create_from_buffer(
            K4A_IMAGE_FORMAT_DEPTH16, width, height,
            width * sizeof(uint16_t), rgbd_buffer->depth_.data_.data(),
            width * height * sizeof(uint16_t), NULL, NULL,
            &k4a_transformed_depth);

        if (K4A_RESULT_SUCCEEDED !=
            k4a_transformation_depth_image_to_color_camera(
                transform, k4a_depth, k4a_transformed_depth)) {
            ErrorLogger::LOG_ERROR("Failed to transform depth frame to color frame at " + std::to_string(_timestamp) + ".", true);
            return nullptr;
        }
    }
    else {
        rgbd_buffer->depth_.Prepare(
            k4a_image_get_width_pixels(k4a_depth),
            k4a_image_get_height_pixels(k4a_depth), 1,
            sizeof(uint16_t));
        memcpy(rgbd_buffer->depth_.data_.data(),
            k4a_image_get_buffer(k4a_depth),
            k4a_image_get_size(k4a_depth));
    }

    DrawObject(rgbd_buffer->depth_);

    /* process depth */
    k4a_image_release(k4a_color);
    k4a_image_release(k4a_depth);
    if (transform) {
        k4a_image_release(k4a_transformed_depth);
    }

    return rgbd_buffer;
}

MKV_Data::MKV_Data(std::string my_folder, std::string mkv_name, std::string calibration_name) : Abstract_Data(my_folder)
{
    ErrorLogger::EXECUTE("Initialization", this, &MKV_Data::Initialize, my_folder, mkv_name, calibration_name);

    ErrorLogger::EXECUTE("Calibrate Camera", this, &MKV_Data::Calibrate);

    ErrorLogger::EXECUTE("Retrieve Playback Data", this, &MKV_Data::GetPlaybackDataRaw);

    ErrorLogger::EXECUTE("Create Intrinsic Tensor", this, &MKV_Data::GetIntrinsicTensor);
    ErrorLogger::EXECUTE("Create Extrinsic Tensor", this, &MKV_Data::GetExtrinsicTensor);

    transform = k4a_transformation_create(&calibration);

    ErrorLogger::EXECUTE("Set Capture To First Frame", this, &MKV_Data::CycleCaptureForwards);

    ErrorLogger::EXECUTE("Decompressing Capture", this, &MKV_Data::DecompressCapture);
}

MKV_Data::~MKV_Data()
{
    if (main_camera_data == this)
    {
        main_camera_data = nullptr;
    }

    if (capture != nullptr)
    {
        k4a_capture_release(*capture);
        delete capture;
    }

    k4a_transformation_destroy(transform);
    k4a_playback_close(handle);
}

uint64_t MKV_Data::GetCaptureTimestamp()
{
    uint64_t min_timestamp = -1;
    k4a_image_t images[3];
    images[0] = k4a_capture_get_color_image(*capture);
    images[1] = k4a_capture_get_depth_image(*capture);
    images[2] = k4a_capture_get_ir_image(*capture);

    for (int i = 0; i < 3; i++)
    {
        if (images[i] != NULL)
        {
            uint64_t timestamp = k4a_image_get_device_timestamp_usec(images[i]);
            if (timestamp < min_timestamp)
            {
                min_timestamp = timestamp;
            }
            k4a_image_release(images[i]);
            images[i] = NULL;
        }
    }

    return min_timestamp;
}

bool MKV_Data::CycleCaptureForwards()
{
    if (capture == nullptr)
    {
        capture = new k4a_capture_t();
    }
    else
    {
        k4a_capture_release(*capture);
    }

    *capture = NULL;

    switch (k4a_playback_get_next_capture(handle, capture))
    {
    case k4a_stream_result_t::K4A_STREAM_RESULT_EOF:
        ErrorLogger::LOG_ERROR("Stream has reached EOF on: " + mkv_file);
        return false;
        break;
    case k4a_stream_result_t::K4A_STREAM_RESULT_FAILED:
        ErrorLogger::LOG_ERROR("Stream failed on: " + mkv_file, true);
        return false;
        break;
    default:
        _timestamp = ErrorLogger::EXECUTE("Retrieving Capture Timestamp", this, &MKV_Data::GetCaptureTimestamp);
        break;
    }

    return true;
}

bool MKV_Data::CycleCaptureBackwards()
{
    if (capture == nullptr)
    {
        capture = new k4a_capture_t();
    }
    else
    {
        k4a_capture_release(*capture);
    }

    *capture = NULL;

    switch (k4a_playback_get_previous_capture(handle, capture))
    {
    case k4a_stream_result_t::K4A_STREAM_RESULT_EOF:
        ErrorLogger::LOG_ERROR("Stream has reached EOF on: " + mkv_file);
        return false;
        break;
    case k4a_stream_result_t::K4A_STREAM_RESULT_FAILED:
        ErrorLogger::LOG_ERROR("Stream failed on: " + mkv_file);
        return false;
        break;
    default:
        _timestamp = ErrorLogger::EXECUTE("Retrieving Capture Timestamp", this, &MKV_Data::GetCaptureTimestamp);
        break;
    }

    return true;
}

bool MKV_Data::SeekToTime(uint64_t time)
{
    if (capture == nullptr)
    {
        capture = new k4a_capture_t();
    }
    else
    {
        k4a_capture_release(*capture);
    }

    *capture = NULL;

    if (k4a_result_t::K4A_RESULT_SUCCEEDED !=
        k4a_playback_seek_timestamp(handle, time, k4a_playback_seek_origin_t::K4A_PLAYBACK_SEEK_DEVICE_TIME))
    {
        ErrorLogger::LOG_ERROR("Problem seeking timestamp on: " + mkv_file);
        return false;
    }

    switch (k4a_playback_get_next_capture(handle, capture))
    {
    case k4a_stream_result_t::K4A_STREAM_RESULT_EOF:
        ErrorLogger::LOG_ERROR("Stream has reached EOF on: " + mkv_file);
        return false;
        break;
    case k4a_stream_result_t::K4A_STREAM_RESULT_FAILED:
        ErrorLogger::LOG_ERROR("Stream failed on: " + mkv_file);
        return false;
        break;
    default:
        _timestamp = ErrorLogger::EXECUTE("Retrieving Capture Timestamp", this, &MKV_Data::GetCaptureTimestamp);
        break;
    }

    std::cout << "Seeking to " << _timestamp << std::endl;

    return true;
}

std::shared_ptr<open3d::geometry::RGBDImage> MKV_Data::GetFrameRGBD()
{
    bool valid_frame = false;

    std::shared_ptr<open3d::geometry::RGBDImage> rgbd;

    while (!valid_frame)
    {
        rgbd = ErrorLogger::EXECUTE("Decompressing Capture", this, &MKV_Data::DecompressCapture);

        valid_frame = (rgbd != nullptr);

        if (!valid_frame)
        {
            ErrorLogger::EXECUTE("Cycling Through Bad Capture", this, &MKV_Data::CycleCaptureForwards);
        }
    }

    return rgbd;
}

open3d::camera::PinholeCameraParameters MKV_Rendering::MKV_Data::GetParameters()
{
    open3d::camera::PinholeCameraParameters to_return;

    to_return.extrinsic_ = extrinsic_mat;

    to_return.intrinsic_ =  open3d::camera::PinholeCameraIntrinsic(
        calibration.color_camera_calibration.resolution_width,
        calibration.color_camera_calibration.resolution_height, 
        calibration.color_camera_calibration.intrinsics.parameters.param.fx,
        calibration.color_camera_calibration.intrinsics.parameters.param.fy,
        calibration.color_camera_calibration.intrinsics.parameters.param.cx,
        calibration.color_camera_calibration.intrinsics.parameters.param.cy
        );

    return to_return;
}

void MKV_Rendering::MKV_Data::WriteIntrinsics(std::string filename_and_path)
{
    std::ofstream file(filename_and_path + ".json", std::ofstream::binary);
    file.write(reinterpret_cast<const char*>(&playback_data[0]), playback_data.size());
    file.close();
}

void MKV_Rendering::MKV_Data::PackIntoVoxelGrid(open3d::t::geometry::TSDFVoxelGrid* grid, VoxelGridData* data)
{
    if (calibration_file == "")
        ErrorLogger::LOG_ERROR("No calibration present on " + folder_name + "!", true);

    auto rgbd = GetFrameRGBD();

    auto color = open3d::t::geometry::Image::FromLegacyImage(rgbd->color_);
    auto depth = open3d::t::geometry::Image::FromLegacyImage(rgbd->depth_);

    color.To(grid->GetDevice());
    depth.To(grid->GetDevice());

    grid->Integrate(depth, color,
        intrinsic_t, extrinsic_t,
        data->depth_scale, data->depth_max);
}

void MKV_Rendering::MKV_Data::PackIntoOldVoxelGrid(open3d::geometry::VoxelGrid* grid)
{
    if (calibration_file == "")
        ErrorLogger::LOG_ERROR("No calibration present on " + folder_name + "!", true);

    auto rgbd = GetFrameRGBD();

    auto params = GetParameters();

    grid->CarveSilhouette(rgbd->depth_, params, true);
}
