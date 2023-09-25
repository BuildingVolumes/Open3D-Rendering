#pragma once

#include "open3d/Open3D.h"

#include "STB_Image_Wrapper.h"

#include <string>
#include <vector>
#include <filesystem>
#include <fstream>

inline bool GetImageOpen3D(std::string filename, open3d::geometry::Image& image)
{
    STB_Image_Wrapper stbiw;

    return stbiw.GetImageOpen3D(filename, image);
}

inline void SplitString(std::string to_split, std::vector<std::string>& destination, std::string delims, std::string ignore_characters = "", bool cull_empty_strings = true)
{
    size_t iterator = 0;
    size_t loc = 0;

    for (int i = 0; i < ignore_characters.length(); ++i)
    {
        auto to_erase = to_split.find(ignore_characters[i]);

        while (to_erase != std::string::npos)
        {
            to_split.erase(to_erase, 1);

            to_erase = to_split.find(ignore_characters[i]);
        }
    }

    while (loc < to_split.size())
    {
        loc = to_split.find_first_of(delims, iterator);
        std::string temp_data = to_split.substr(iterator, loc - iterator);

        if (!cull_empty_strings || (temp_data.size() > 0))
        {
            destination.push_back(temp_data);
        }

        iterator = loc + 1;
    }
}

inline std::vector<std::string> GetDirectories(const std::string& s)
{
    std::vector<std::string> r;
    for (auto& p : std::filesystem::directory_iterator(s))
        if (p.is_directory())
            r.push_back(p.path().string());
    return r;
}

inline std::vector<std::string> GetFiles(const std::string& s)
{
    std::vector<std::string> r;
    for (auto& p : std::filesystem::directory_iterator(s))
        if (p.is_regular_file())
            r.push_back(p.path().string());
    return r;
}

inline void DebugVector(std::vector<std::string>& vec)
{
    for (int i = 0; i < vec.size(); ++i)
    {
        std::cout << std::to_string(i) << ": " << vec[i] << std::endl;
    }
}

inline void WriteError(std::exception &e, std::string filename, std::string otherData = "")
{
    std::fstream emptyTXT;

    std::string what = e.what();
    what += "\n";

    emptyTXT.open(filename, std::ios_base::out);

    emptyTXT.write(what.c_str(), what.length());
    emptyTXT.write(otherData.c_str(), otherData.length());
    emptyTXT.close();
}

inline void Dbg(std::string msg)
{
    std::cout << msg << std::endl;
    system("pause");
}

inline std::string GetNumberFixedLength(int num, int length)
{
    std::string to_return;
    std::string iter_num = std::to_string(num);
    to_return.resize(length - iter_num.size(), '0');
    to_return.append(iter_num);

    return to_return;
}

inline std::string RemoveFileExtention(std::string filename)
{
    auto loc = filename.rfind('.');

    if (loc == std::string::npos)
    {
        return filename;
    }
    
    return filename.substr(0, loc);
}