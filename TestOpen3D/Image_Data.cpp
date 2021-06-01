#include "Image_Data.h"

MKV_Rendering::Image_Data::Image_Data(std::string root_folder, std::string color_folder, std::string depth_folder, double FPS) : Abstract_Data(root_folder)
{
    this->color_folder = root_folder + "/" + color_folder;
    this->depth_folder = root_folder + "/" + depth_folder;
    this->FPS = FPS;
    current_frame = 0;

    open3d::utility::filesystem::ListFilesInDirectory(this->color_folder, color_files);
    std::sort(color_files.begin(), color_files.end());

    open3d::utility::filesystem::ListFilesInDirectory(this->depth_folder, depth_files);
    std::sort(depth_files.begin(), depth_files.end());
}

MKV_Rendering::Image_Data::~Image_Data()
{
}

uint64_t MKV_Rendering::Image_Data::GetCaptureTimestamp()
{
    uint64_t to_return = (current_frame * 1000000.0 / FPS);


    return (current_frame * 1000000.0 / FPS);
}

void MKV_Rendering::Image_Data::CycleCaptureForwards()
{
    ++current_frame;
}

void MKV_Rendering::Image_Data::CycleCaptureBackwards()
{
    --current_frame;
}

void MKV_Rendering::Image_Data::SeekToTime(uint64_t time)
{
    current_frame = time * FPS;
}

void MKV_Rendering::Image_Data::PackIntoVoxelGrid(open3d::t::geometry::TSDFVoxelGrid* grid, VoxelGridData* data)
{
    auto color = (*open3d::t::io::CreateImageFromFile(color_files[current_frame]));
    auto depth = (*open3d::t::io::CreateImageFromFile(depth_files[current_frame]));

    color.To(grid->GetDevice());
    depth.To(grid->GetDevice());

    grid->Integrate(depth, color,
        intrinsic_t, extrinsic_t,
        data->depth_scale, data->depth_max);
}
