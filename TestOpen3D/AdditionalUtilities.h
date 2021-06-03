#pragma once

#include <string>
#include <vector>
#include <filesystem>

inline void SplitString(std::string to_split, std::vector<std::string>& destination, char delim, bool cull_empty_strings = true)
{
    size_t iterator = 0;
    size_t loc = 0;

    while (loc < to_split.size())
    {
        loc = to_split.find(delim, iterator);
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