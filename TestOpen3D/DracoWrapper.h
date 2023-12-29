#pragma once

#include <string>

class DracoWrapper
{
public:
	void MassCompressMeshes(std::string root_folder, std::string output_folder, std::string input_tag, std::string output_name, std::string output_extension);

	void MassDecompressMeshes(std::string root_folder, std::string output_folder, std::string input_tag, std::string output_name, std::string output_extension);
};