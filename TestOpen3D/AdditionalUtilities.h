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
