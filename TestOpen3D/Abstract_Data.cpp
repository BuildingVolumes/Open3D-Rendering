#include "Abstract_Data.h"

MKV_Rendering::Abstract_Data::Abstract_Data(std::string my_folder)
{
	folder_name = my_folder;
}

open3d::core::Tensor MKV_Rendering::Abstract_Data::GetIntrinsic()
{
	return intrinsic_t;
}

open3d::core::Tensor MKV_Rendering::Abstract_Data::GetExtrinsic()
{
	return extrinsic_t;
}
