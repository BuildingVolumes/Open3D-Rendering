#pragma once

#include <string>
#include "open3d/Open3D.h"

class STB_Image_Wrapper
{
public:
	bool GetImageOpen3D(std::string filename, open3d::geometry::Image& image);
};