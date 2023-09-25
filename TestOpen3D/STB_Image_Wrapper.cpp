#include "STB_Image_Wrapper.h"

//#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

bool STB_Image_Wrapper::GetImageOpen3D(std::string filename, open3d::geometry::Image& image)
{
    if (!open3d::io::ReadImageFromPNG(filename, image))
    {
        std::cout << "IMAGE NOT FOUND!" << std::endl;
        return false;
    }

    int x, y, n;
    //auto data = stbi_load(filename.c_str(), &x, &y, &n, 0);
    
    auto data = stbi_load_16(filename.c_str(), &x, &y, &n, 4);


    if (data != nullptr)
    {
        //auto num_data = image.data_.size();
        auto num_data = x * y * n * 2;

        uint8_t* d = (uint8_t*)&(data[0]);

        for (int i = 0; i < 100; ++i)
        {
            std::cout << (int)d[i] << std::endl;
        }

        system("pause");

        memcpy(image.data_.data(), data, num_data);

        return true;
    }
    else
    {
        std::cout << "INVALID IMAGE!" << std::endl;
        return false;
    }
}
