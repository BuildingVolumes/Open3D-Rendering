#pragma once

#include "open3d/Open3D.h"

namespace EncDec {
	class EncodeDecodeManager
	{
    public:
        void split_int(int& shift_x, int& shift_y, int index);

        void combine_int(int shift_x, int shift_y, int& return_value);


        std::shared_ptr<open3d::geometry::Image> DefaultEncodeOBJ(std::shared_ptr<open3d::geometry::TriangleMesh> mesh, int image_channels, int image_bytes_per_channel);

        std::shared_ptr<open3d::geometry::TriangleMesh> DefaultDecodeOBJ(std::shared_ptr<open3d::geometry::Image> image);


        void write_morton_single_simplified(byte* _dest, const byte* _src, int data_count, int bytes_per_line, int bytes_per_pixel);

        std::shared_ptr<open3d::geometry::Image> MortonEncodeOBJ(std::shared_ptr<open3d::geometry::TriangleMesh> mesh, int image_channels, int image_bytes_per_channel);

        void read_morton_single_simplified(byte* _dest, const byte* _src, int byte_count, int bytes_per_line, int bytes_per_pixel);

        std::shared_ptr<open3d::geometry::TriangleMesh> MortonDecodeOBJ(std::shared_ptr<open3d::geometry::Image> image);


        bool FFT();
    };
}