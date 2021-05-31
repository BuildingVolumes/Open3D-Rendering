#include "Image_Data.h"

MKV_Rendering::Image_Data::Image_Data(std::string root_folder, std::string color_folder, std::string depth_folder, double FPS) : Abstract_Data(root_folder)
{
    this->color_folder = root_folder + "/" + color_folder;
    this->depth_folder = root_folder + "/" + depth_folder;
    this->FPS = FPS;
    currentFrame = 0;

    std::vector<std::string> color_filenames;
    open3d::utility::filesystem::ListFilesInDirectory(this->color_folder, color_filenames);
    std::sort(color_filenames.begin(), color_filenames.end());

    std::vector<std::string> depth_filenames;
    open3d::utility::filesystem::ListFilesInDirectory(this->depth_folder, depth_filenames);
    std::sort(depth_filenames.begin(), depth_filenames.end());
}

MKV_Rendering::Image_Data::~Image_Data()
{
}

uint64_t MKV_Rendering::Image_Data::GetCaptureTimestamp()
{
    return (currentFrame * 1000000.0 / FPS);
}

void MKV_Rendering::Image_Data::CycleCaptureForwards()
{
    ++currentFrame;
}

void MKV_Rendering::Image_Data::CycleCaptureBackwards()
{
    --currentFrame;
}

void MKV_Rendering::Image_Data::SeekToTime(uint64_t time)
{
    currentFrame = time * FPS;
}

void MKV_Rendering::Image_Data::PackIntoVoxelGrid(open3d::t::geometry::TSDFVoxelGrid* grid, VoxelGridData* data)
{
}
