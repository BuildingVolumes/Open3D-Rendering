#pragma once

#include <vector>
#include <string>

class VolumetricStreamStruct
{
public:
	std::string root_folder;
	std::vector<std::string> mesh_names;
	std::vector<std::string> texture_names;
};