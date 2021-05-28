#pragma once

#include <string>

namespace MKV_Rendering {
    struct VoxelGridData
    {
        int blocks = 1000; //May need to change
        float voxel_size = 3.f / 512.f; //May need to change
        float depth_scale = 1000.f; //May need to change
        float depth_max = 3.f; //May need to change
        float signed_distance_field_truncation = 0.04f; //May need to change

        std::string device_code = "CPU:0"; //May need to change, but probably not
    };
}