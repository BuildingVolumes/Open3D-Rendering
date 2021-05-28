// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include "open3d/Open3D.h"
#include "open3d/io/sensor/azure_kinect/MKVMetadata.h"
#include <fstream>
#include <json/json.h>
#include "open3d/io/sensor/azure_kinect/K4aPlugin.h"
#include <memory>
#include "open3d/geometry/RGBDImage.h"
#include <turbojpeg.h>

#include "ErrorLogger.h"

using namespace open3d;
using namespace open3d::core;
using namespace open3d::io;
using namespace Eigen;

struct GridData
{
    int blocks = 1000; //May need to change
    float voxel_size = 3.f / 512.f; //May need to change
    float depth_scale = 1000.f; //May need to change
    float depth_max = 3.f; //May need to change
    float signed_distance_field_truncation = 0.04f; //May need to change
};

void Dbg(std::string msg)
{
    std::cout << msg << std::endl;
    system("pause");
}

size_t get_file_length(std::ifstream &filestream)
{
    filestream.seekg(0, std::ios::end);
    size_t file_length = filestream.tellg();
    filestream.seekg(0, std::ios::beg);

    return file_length;
}

void ConvertBGRAToRGB(geometry::Image& bgra, geometry::Image& rgb) {
    if (bgra.bytes_per_channel_ != 1) {
        utility::LogError("BGRA input image must have 1 byte per channel.");
    }
    if (rgb.bytes_per_channel_ != 1) {
        utility::LogError("RGB output image must have 1 byte per channel.");
    }
    if (bgra.num_of_channels_ != 4) {
        utility::LogError("BGRA input image must have 4 channels.");
    }
    if (rgb.num_of_channels_ != 3) {
        utility::LogError("RGB output image must have 3 channels.");
    }
    if (bgra.width_ != rgb.width_ || bgra.height_ != rgb.height_) {
        utility::LogError(
            "BGRA input image and RGB output image have different "
            "dimensions.");
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

class CameraDataMKV
{
public:
    CameraDataMKV* main_camera = nullptr;

    k4a_capture_t* capture = nullptr;

    uint64_t _timestamp = 0;

    k4a_calibration_t _calib;
    k4a_transformation_t _transform = NULL;

    std::string mkv_name;

    float _depth_scale = 0;
    float _depth_max = 0;

    Tensor _intrinsic_t;
    Tensor _extrinsic_t;
    Matrix4d _extrinsic_m;

    k4a_playback_t handle;

    k4a_record_configuration_t record_config;

    std::string extrinsic_path;

    float focalX = 0, focalY = 0, principleX = 0, principleY = 0;

    CameraDataMKV(std::string mkv_file) : mkv_name(mkv_file)
    {
        if (k4a_result_t::K4A_RESULT_SUCCEEDED !=
            k4a_playback_open(mkv_name.c_str(), &handle))
        {
            Dbg("Failed to open file: " + mkv_file);
        }

        if (k4a_result_t::K4A_RESULT_SUCCEEDED !=
            k4a_playback_get_record_configuration(handle, &record_config))
        {
            Dbg("Failed to get record configuration from: " + mkv_file);
        }

        switch (record_config.wired_sync_mode)
        {
        case K4A_WIRED_SYNC_MODE_MASTER:
            if (main_camera == nullptr)
            {
                main_camera = this;
                std::cout << mkv_file << " set as main camera" << std::endl;
            }
            else
            {
                Dbg("Conflict between " + main_camera->mkv_name + " and " + mkv_file + " over main camera");
            }
            break;
        case K4A_WIRED_SYNC_MODE_SUBORDINATE:
            std::cout << mkv_file << " set as subordinate camera" << std::endl;
            break;
        default:
            Dbg("Bad record configuration on: " + mkv_file);
        }

        get_playback_data_RAW();

        _transform = k4a_transformation_create(&_calib);

        cycle_capture();
    }

    ~CameraDataMKV()
    {
        if (main_camera == this)
        {
            main_camera = nullptr;
        }

        if (capture != nullptr)
        {
            k4a_capture_release(*capture);
            delete capture;
        }

        k4a_transformation_destroy(_transform);
        k4a_playback_close(handle);
    }

    k4a_transformation_t get_transform() { return _transform; }

    void get_playback_data_RAW()
    {
        std::vector<uint8_t> playback_data;

        size_t data_len = 0;

        k4a_playback_get_raw_calibration(handle, nullptr, &data_len);

        playback_data.resize(data_len);

        std::cout << "Data size: " << data_len << std::endl;

        switch (k4a_playback_get_raw_calibration(handle, &playback_data[0], &data_len))
        {
        case k4a_buffer_result_t::K4A_BUFFER_RESULT_TOO_SMALL:
            Dbg("Buffer was to small in: " + mkv_name);
            break;
        case k4a_buffer_result_t::K4A_BUFFER_RESULT_FAILED:
            Dbg("Failed to make playback calibration: " + mkv_name);
            break;
        default:
            break;
        }

        if (k4a_result_t::K4A_RESULT_SUCCEEDED !=
            k4a_calibration_get_from_raw(
                (char*)playback_data.data(), playback_data.size() + 1, 
                k4a_depth_mode_t::K4A_DEPTH_MODE_NFOV_UNBINNED,
                k4a_color_resolution_t::K4A_COLOR_RESOLUTION_1080P, &_calib
            ))
        {
            Dbg("Failed to turn raw data into calibration");
        }

        auto params = _calib.color_camera_calibration.intrinsics.parameters.param;

        focalX = params.fx;
        focalY = params.fy;
        principleX = params.cx;
        principleY = params.cy;

        _intrinsic_t = Tensor::Init<double>({
            {params.fx, 0, params.cx},
            {0, params.fy, params.cy},
            {0, 0, 1}
            });
    }

    void cycle_capture()
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
            Dbg("Stream has reached EOF on: " + mkv_name);
            break;
        case k4a_stream_result_t::K4A_STREAM_RESULT_FAILED:
            Dbg("Stream failed on: " + mkv_name);
            break;
        default:
            _timestamp = first_capture_timestamp();
            break;
        }
    }

    const k4a_capture_t* get_capture()
    {
        return capture;
    }

    void print_capture_info()
    {
        k4a_image_t images[3];
        images[0] = k4a_capture_get_color_image(*capture);
        images[1] = k4a_capture_get_depth_image(*capture);
        images[2] = k4a_capture_get_ir_image(*capture);

        printf("%-32s", mkv_name.c_str());
        for (int j = 0; j < 3; j++)
        {
            if (images[j] != NULL)
            {
                uint64_t timestamp = k4a_image_get_device_timestamp_usec(images[j]);
                printf("  %7ju usec", timestamp);
                k4a_image_release(images[j]);
                images[j] = NULL;
            }
            else
            {
                printf("  %12s", "");
            }
        }
        printf("\n");
    }

    uint64_t first_capture_timestamp()
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

    uint64_t get_timestamp_cached() { return _timestamp; }

    std::shared_ptr<open3d::geometry::RGBDImage> get_rgbd_frame()
    {
        bool valid_frame = false;

        std::shared_ptr<open3d::geometry::RGBDImage> rgbd;

        while (!valid_frame)
        {
            rgbd = DecompressCapture(capture, _transform);

            valid_frame = (rgbd != nullptr);

            if (!valid_frame)
            {
                cycle_capture();
            }
        }

        return rgbd;
    }

    void get_rgbd_frame(geometry::Image &_color_ref, geometry::Image &_depth_ref)
    {
        bool valid_frame = false;

        while (!valid_frame)
        {
            auto rgbd = DecompressCapture(capture, _transform);

            valid_frame = (rgbd != nullptr);

            if (!valid_frame)
            {
                cycle_capture();
            }
            else
            {
                _color_ref = rgbd->color_;
                _depth_ref = rgbd->depth_;
            }
        }
    }

    std::shared_ptr<geometry::RGBDImage> DecompressCapture(
        k4a_capture_t* capture, k4a_transformation_t transformation) {
        static std::shared_ptr<geometry::Image> color_buffer = nullptr;
        static std::shared_ptr<geometry::RGBDImage> rgbd_buffer = nullptr;

        if (color_buffer == nullptr) {
            color_buffer = std::make_shared<geometry::Image>();
        }
        if (rgbd_buffer == nullptr) {
            rgbd_buffer = std::make_shared<geometry::RGBDImage>();
        }

        k4a_image_t k4a_color = k4a_capture_get_color_image(*capture);
        k4a_image_t k4a_depth = k4a_capture_get_depth_image(*capture);
        if (k4a_color == nullptr || k4a_depth == nullptr) {
            utility::LogDebug("Skipping empty captures.");
            return nullptr;
        }

        /* Process color */
        if (K4A_IMAGE_FORMAT_COLOR_MJPG !=
            k4a_image_get_format(k4a_color)) {
            utility::LogWarning(
                "Unexpected image format. The stream may have been corrupted.");
            return nullptr;
        }

        int width = k4a_image_get_width_pixels(k4a_color);
        int height = k4a_image_get_height_pixels(k4a_color);

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
            utility::LogWarning("Failed to decompress color image.");
            return nullptr;
        }
        tjDestroy(tjHandle);
        ConvertBGRAToRGB(*color_buffer, rgbd_buffer->color_);

        /* transform depth to color plane */
        k4a_image_t k4a_transformed_depth = nullptr;
        if (transformation) {
            rgbd_buffer->depth_.Prepare(width, height, 1, sizeof(uint16_t));
            k4a_image_create_from_buffer(
                K4A_IMAGE_FORMAT_DEPTH16, width, height,
                width * sizeof(uint16_t), rgbd_buffer->depth_.data_.data(),
                width * height * sizeof(uint16_t), NULL, NULL,
                &k4a_transformed_depth);
            if (K4A_RESULT_SUCCEEDED !=
                k4a_transformation_depth_image_to_color_camera(
                    transformation, k4a_depth, k4a_transformed_depth)) {
                utility::LogWarning(
                    "Failed to transform depth frame to color frame.");
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

        /* process depth */
        k4a_image_release(k4a_color);
        k4a_image_release(k4a_depth);
        if (transformation) {
            k4a_image_release(k4a_transformed_depth);
        }

        return rgbd_buffer;
    }
};

void DrawGeo(geometry::Image image)
{
    std::vector<std::shared_ptr<const geometry::Geometry>> toDraw;

    auto to_draw = std::make_shared<geometry::Image>(
        image);

    toDraw.push_back(to_draw);

    visualization::DrawGeometries(toDraw);
}

void split_string(std::string to_split, std::vector<std::string>& destination, char delim, bool cull_empty_strings = true)
{
    size_t iterator = 0;

    while (iterator < to_split.size())
    {
        auto loc = to_split.find(delim, iterator);
        std::string temp_data = to_split.substr(iterator, loc - iterator);

        if (cull_empty_strings && (temp_data.size() > 0))
        {
            destination.push_back(temp_data);
        }

        iterator = loc + 1;
    }
}

Tensor build_extrinsic_matrix_default()
{
    return core::eigen_converter::EigenMatrixToTensor(Eigen::Projective3d::Identity().matrix());
}

Tensor build_extrinsic_matrix_kinect(std::string extrinsic_path)
{
    std::string file_contents;
    std::ifstream calib_file = std::ifstream(extrinsic_path);

    while (!calib_file.eof())
    {
        std::string single_content = "";
        std::getline(calib_file, single_content);
        single_content += " ";

        file_contents.append(single_content);
    }

    Vector3d translation;
    Matrix3d r_mat_3 = Matrix3d::Identity();

    std::vector<std::string> extrinsic_data;
    split_string(file_contents, extrinsic_data, ' ');

    for (int j = 0; j < 3; ++j)
    {
        translation[j] = std::stof(extrinsic_data[j]);
    }

    for (int j = 0; j < 9; ++j)
    {
       r_mat_3(j / 3, j % 3) = std::stof(extrinsic_data[j + 3]);
    }

    Matrix4d final_mat = Matrix4d::Identity();

    final_mat.block<3, 3>(0, 0) = r_mat_3;
    final_mat.block<3, 1>(0, 3) = r_mat_3 * translation;

    final_mat = final_mat.inverse();

    return core::eigen_converter::EigenMatrixToTensor(final_mat);
}

Tensor build_extrinsic_matrix_kinect(std::string extrinsic_path, Matrix4d &to_write)
{
    std::string file_contents;
    std::ifstream calib_file = std::ifstream(extrinsic_path);

    while (!calib_file.eof())
    {
        std::string single_content = "";
        std::getline(calib_file, single_content);
        single_content += " ";

        file_contents.append(single_content);
    }

    Vector3d translation;
    Matrix3d r_mat_3 = Matrix3d::Identity();

    std::vector<std::string> extrinsic_data;
    split_string(file_contents, extrinsic_data, ' ');

    for (int j = 0; j < 3; ++j)
    {
        translation[j] = std::stof(extrinsic_data[j]);
    }

    for (int j = 0; j < 9; ++j)
    {
        r_mat_3(j / 3, j % 3) = std::stof(extrinsic_data[j + 3]);
    }

    to_write = Matrix4d::Identity();

    to_write.block<3, 3>(0, 0) = r_mat_3;
    to_write.block<3, 1>(0, 3) = r_mat_3 * translation;

    to_write = to_write.inverse();

    return core::eigen_converter::EigenMatrixToTensor(to_write);
}

Tensor build_intrinsic_matrix_default()
{
    camera::PinholeCameraIntrinsic intrinsic = camera::PinholeCameraIntrinsic(
        camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault);

    auto focal_length = intrinsic.GetFocalLength();
    auto principal_point = intrinsic.GetPrincipalPoint();
    return Tensor::Init<double>(
        { {focal_length.first, 0, principal_point.first},
         {0, focal_length.second, principal_point.second},
         {0, 0, 1} });
}

Tensor build_intrinsic_matrix_kinect(std::string intrinsic_path)
{
    std::ifstream intrinsic_file = std::ifstream(intrinsic_path);
    std::string file_contents;

    //Get the length of the data
    intrinsic_file.seekg(0, std::ios::end);
    size_t file_length = intrinsic_file.tellg();
    intrinsic_file.seekg(0, std::ios::beg);

    file_contents.resize(file_length);
    intrinsic_file.read(&file_contents[0], file_length);

    intrinsic_file.close();

    //Save the kinect's camera data
    k4a_calibration_t calib;

    k4a_result_t res = k4a_calibration_get_from_raw(&file_contents[0], file_contents.size() + 1, k4a_depth_mode_t::K4A_DEPTH_MODE_NFOV_UNBINNED, k4a_color_resolution_t::K4A_COLOR_RESOLUTION_1080P, &calib);
    if (res == k4a_result_t::K4A_RESULT_FAILED)
    {
        std::cout << "ERROR: FAILED TO READ INTRINSICS JSON!" << std::endl;
    }

    //Intrinsic parameters, of the depth camera because that's the important one
    auto int_par = calib.depth_camera_calibration.intrinsics.parameters;

    return Tensor::Init<double>({
            {int_par.param.fx, 0, int_par.param.cx},
            {0, int_par.param.fy, int_par.param.cy},
            {0, 0, 1}
        });
}

t::geometry::TSDFVoxelGrid CreateVoxelGrid(core::Device& device, int blocks = 1000, float voxel_size = 3.f / 512.f, float depth_scale = 1000.f, float depth_max = 3.f, float sdf_trunc = 0.04f)
{
    return t::geometry::TSDFVoxelGrid(

        {
            {"tsdf", core::Dtype::Float32},
            {"weight", core::Dtype::UInt16},
            {"color", core::Dtype::UInt16}
        },

        voxel_size, sdf_trunc, 16, blocks, device
    );
}

pipelines::integration::ScalableTSDFVolume CreateTSDFVolume(double voxel_length, double signed_distance_field_truncation, pipelines::integration::TSDFVolumeColorType color_type, int volume_resolution = 16, int depth_stride = 4)
{
    return pipelines::integration::ScalableTSDFVolume(
        voxel_length, signed_distance_field_truncation, color_type, volume_resolution, depth_stride
    );
}

void read_frame_from_MKV(pipelines::integration::ScalableTSDFVolume& tsdf_volume, std::vector<CameraDataMKV*>& cam_data)
{
    for (auto cam : cam_data)
    {
        auto im = cam->get_rgbd_frame();

        auto intrinsic = camera::PinholeCameraIntrinsic();
        im->depth_ = *im->depth_.ConvertDepthToFloatImage();

        auto width = im->color_.width_;
        auto height = im->color_.height_;

        intrinsic.SetIntrinsics(width, height, cam->focalX, cam->focalY, cam->principleX, cam->principleY);

        tsdf_volume.Integrate(*im, intrinsic, cam->_extrinsic_m);
    }
}

void read_frame_from_MKV(t::geometry::TSDFVoxelGrid& grid, core::Device& device, std::vector<CameraDataMKV*> &cam_data)
{
    for (auto cam : cam_data)
    {
        geometry::Image color;
        geometry::Image depth;

        cam->get_rgbd_frame(color, depth);

        t::geometry::Image true_color = t::geometry::Image::FromLegacyImage(color);
        t::geometry::Image true_depth = t::geometry::Image::FromLegacyImage(depth);

        true_color.To(device);
        true_depth.To(device);

        std::cout << true_color.ToString() << std::endl;
        std::cout << true_depth.ToString() << std::endl;

        std::cout << cam->_intrinsic_t.ToString() << std::endl;
        std::cout << cam->_extrinsic_t.ToString() << std::endl;

        grid.Integrate(true_depth, true_color, cam->_intrinsic_t, cam->_extrinsic_t, cam->_depth_scale, cam->_depth_max);
    }
}

void read_frame_from_MKV_and_draw(std::vector<CameraDataMKV*>& cam_data, Vector3d origin, Vector3d color, double voxel_size, Vector3d xyz_dimensions, bool keep_voxels_outside_image)
{
    geometry::VoxelGrid grid;

    std::cout << "Creating grid at " << origin << " with voxel size " << voxel_size << " and dimensions " << xyz_dimensions << std::endl;

    grid.CreateDense(origin, color, voxel_size, xyz_dimensions[0], xyz_dimensions[1], xyz_dimensions[2]);

    for (auto cam : cam_data)
    {
        std::cout << "Rendering frame from " << cam->mkv_name << "..." << std::endl;
        auto im = cam->get_rgbd_frame();
        auto width = im->depth_.width_;
        auto height = im->depth_.height_;

        auto intrinsic = camera::PinholeCameraIntrinsic();
        intrinsic.SetIntrinsics(width, height, cam->focalX, cam->focalY, cam->principleX, cam->principleY);

        auto pinholeParams = camera::PinholeCameraParameters();
        pinholeParams.extrinsic_ = cam->_extrinsic_m;
        pinholeParams.intrinsic_ = intrinsic;

        grid.CarveDepthMap(im->depth_, pinholeParams, keep_voxels_outside_image);
    }

    auto grid_ptr = std::make_shared<geometry::VoxelGrid>(
        grid);

    std::vector<std::shared_ptr<const geometry::Geometry>> toDraw;
    toDraw.push_back(grid_ptr);

    visualization::DrawGeometries(toDraw);
}

void read_frame_from_MKV_and_draw(GridData &gd, core::Device& device, std::vector<CameraDataMKV*>& cam_data)
{
    t::geometry::TSDFVoxelGrid grid = CreateVoxelGrid(device, gd.blocks, gd.voxel_size, gd.depth_scale, gd.depth_max, gd.signed_distance_field_truncation);

    for (auto cam : cam_data)
    {
        geometry::Image color;
        geometry::Image depth;

        cam->get_rgbd_frame(color, depth);

        t::geometry::Image true_color = t::geometry::Image::FromLegacyImage(color);
        t::geometry::Image true_depth = t::geometry::Image::FromLegacyImage(depth);

        true_color.To(device);
        true_depth.To(device);

        grid.Integrate(true_depth, true_color, cam->_intrinsic_t, cam->_extrinsic_t, cam->_depth_scale, cam->_depth_max);
    }

    auto mesh = grid.ExtractSurfaceMesh(0.0f);
    auto mesh_legacy = std::make_shared<geometry::TriangleMesh>(
        mesh.ToLegacyTriangleMesh());

    std::vector<std::shared_ptr<const geometry::Geometry>> toDraw;
    toDraw.push_back(mesh_legacy);

    visualization::DrawGeometries(toDraw);
}

void bake_frame_into_separate_meshes_MKV(GridData &gd, core::Device& device, std::vector<CameraDataMKV*>& cam_data, std::string mesh_name, std::string mesh_file_type)
{
    int iter = 0;

    for (auto cam : cam_data)
    {
        auto fresh_grid = CreateVoxelGrid(device, gd.blocks, gd.voxel_size, gd.depth_scale, gd.depth_max, gd.signed_distance_field_truncation);

        geometry::Image color;
        geometry::Image depth;

        cam->get_rgbd_frame(color, depth);

        std::cout << cam->get_timestamp_cached() << std::endl;

        t::geometry::Image true_color = t::geometry::Image::FromLegacyImage(color);
        t::geometry::Image true_depth = t::geometry::Image::FromLegacyImage(depth);

        true_color.To(device);
        true_depth.To(device);

        fresh_grid.Integrate(true_depth, true_color, cam->_intrinsic_t, cam->_extrinsic_t, cam->_depth_scale, cam->_depth_max);

        auto mesh = fresh_grid.ExtractSurfaceMesh(0.0f);
        auto mesh_legacy = std::make_shared<geometry::TriangleMesh>(
            mesh.ToLegacyTriangleMesh());
        open3d::io::WriteTriangleMesh(mesh_name + std::to_string(iter) + mesh_file_type,
            *mesh_legacy);

        ++iter;
    }
}

//Create a mesh from one frame of all azure kinect camera
void create_kinect_mesh()
{
    using MaskCode = t::geometry::TSDFVoxelGrid::SurfaceMaskCode;

    //Folder paths, parameters and whatnot
    std::string main_image_directory = "Kinect Test 1";
    std::string mkv_file_name = "rgbd_video.mkv";
    std::string extrinsics_file_name = "calib.log";

    //Voxel grid specifics
    GridData gd;

    //Select a device to use for the voxel grid
    std::string device_code = "CPU:0"; //May need to change, but probably not
    core::Device device(device_code);

    std::string mesh_file_name = "kinect test 1 mesh.ply"; //Definitely need to change

    //Folders to find information from
    std::vector<std::string> video_folders;
    video_folders.push_back("FramesCam0");
    video_folders.push_back("FramesCam1");
    video_folders.push_back("FramesCam2");
    video_folders.push_back("FramesCam3");
    video_folders.push_back("FramesCam4");

    //Vector of camera objects
    std::vector<CameraDataMKV*> camera_data;

    size_t highest_ms = 0; // we can control what microsecond to start the video at here
    highest_ms = 10900000; //This would be 11 seconds in

    //Set data for each camera
    for (int i = 0; i < video_folders.size(); ++i)
    {   
        std::cout << "\nParsing for camera data in " << video_folders[i] << "..." << std::endl;

        CameraDataMKV* cdm = new CameraDataMKV(main_image_directory + "/" + video_folders[i] + "/" + mkv_file_name);

        cdm->extrinsic_path = main_image_directory + "/" + video_folders[i] + "/" + extrinsics_file_name;

        cdm->_extrinsic_t = build_extrinsic_matrix_kinect(cdm->extrinsic_path, cdm->_extrinsic_m);

        //Depth information
        cdm->_depth_scale = gd.depth_scale;
        cdm->_depth_max = gd.depth_max;

        camera_data.push_back(cdm);

        if (highest_ms < cdm->get_timestamp_cached())
        {
            highest_ms = cdm->get_timestamp_cached();
        }
    }

    for (auto cam : camera_data)
    {
        while (cam->get_timestamp_cached() < highest_ms)
        {
            cam->cycle_capture();
        }

        std::cout << "Found capture! " << cam->get_timestamp_cached() << std::endl;
    }

    //Grid to render a mesh
    t::geometry::TSDFVoxelGrid voxel_grid = CreateVoxelGrid(device, gd.blocks, gd.voxel_size, gd.depth_scale, gd.depth_max, gd.signed_distance_field_truncation);
    //pipelines::integration::ScalableTSDFVolume tsdf_volume = CreateTSDFVolume(3.0 / 512.0, 0.04, pipelines::integration::TSDFVolumeColorType::RGB8, 16, 4);

    //Here we render a single frame
    read_frame_from_MKV(voxel_grid, device, camera_data);
    //read_frame_from_MKV_and_draw(camera_data, Vector3d(0, 0, 0), Vector3d(1, 0, 0), gd.voxel_size, 200.0 * gd.voxel_size * Vector3d(1, 1, 1), true);

    //read_frame_from_MKV(tsdf_volume, camera_data);
    //bake_frame_into_separate_meshes_MKV(gd, device, camera_data, "Voxel Mesh", ".obj");

    //Construct a mesh from the data, print to a file
    auto mesh = voxel_grid.ExtractSurfaceMesh(0.0f);
    auto mesh_legacy = std::make_shared<geometry::TriangleMesh>(
        mesh.ToLegacyTriangleMesh());
    open3d::io::WriteTriangleMesh(mesh_file_name,
        *mesh_legacy);

    //auto mesh_legacy = tsdf_volume.ExtractTriangleMesh();
    
    //Allow us to view the mesh after creating
    std::vector<std::shared_ptr<const geometry::Geometry>> toDraw;
    toDraw.push_back(mesh_legacy);
    
    visualization::DrawGeometries(toDraw);

    while (!camera_data.empty())
    {
        delete camera_data[camera_data.size() - 1];

        camera_data.pop_back();
    }
}

//int main() {
//    //create_kinect_mesh();
//
//    return 0;
//}
