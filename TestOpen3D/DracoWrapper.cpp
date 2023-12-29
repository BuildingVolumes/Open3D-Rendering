#include "DracoWrapper.h"

#include "AdditionalUtilities.h"

void DracoWrapper::MassCompressMeshes(std::string root_folder, std::string output_folder, std::string input_tag, std::string output_name, std::string output_extension)
{
	auto allFiles = GetFiles(root_folder);

	std::vector<std::string> filenames;

	std::cout << "Starting..." << std::endl;

	for (int i = 0; i < allFiles.size(); ++i)
	{
		if (allFiles[i].find(input_tag) != std::string::npos)
		{
			filenames.push_back(allFiles[i]);
		}

		//std::vector<std::string> splitString;
		//
		//SplitString(allFiles[i], splitString, ".");
		//
		////std::cout << "Found file: " << splitString.back() << std::endl;
		//
		//if (!std::strcmp(StringLowerCase(splitString.back()).c_str(), "obj"))
		//{
		//	filenames.push_back(allFiles[i]);
		//
		//	//std::cout << "Match " << filenames.size() << ": " << filenames.back() << std::endl;
		//}
	}

	//return;

	std::string input_command;

	std::string com_header = "D:\\_VV_COMPRESSION_SOFTWARES\\_DRACO\\draco-master\\_BUILD\\Release\\draco_encoder";

	std::string input_mesh;

	std::string output_number;

	//std::string output_tag = "CompressedMeshDRACO";

	//std::string output_extension = ".drc";

	std::string complete_output;

	for (int i = 0; i < filenames.size(); ++i)
	{
		input_mesh = filenames[i];

		output_number = GetNumberFixedLength(i, 6);

		complete_output = output_folder + "\\" + output_name + "_" + output_number + output_extension;

		input_command = com_header + " -i " + filenames[i] + " -o " + complete_output;

		//std::cout << input_command.c_str() << std::endl;
 
		//continue;

		system(input_command.c_str());
	}

	//std::cout << "Found files: " << std::endl;
	//
	//for (int i = 0; i < filenames.size(); ++i)
	//{
	//	std::cout << filenames[i] << std::endl;
	//}
}

void DracoWrapper::MassDecompressMeshes(std::string root_folder, std::string output_folder, std::string input_tag, std::string output_name, std::string output_extension)
{
	auto allFiles = GetFiles(root_folder);

	std::vector<std::string> filenames;

	std::cout << "Starting..." << std::endl;

	for (int i = 0; i < allFiles.size(); ++i)
	{
		if (allFiles[i].find(input_tag) != std::string::npos)
		{
			filenames.push_back(allFiles[i]);
		}

		//std::vector<std::string> splitString;
		//
		//SplitString(allFiles[i], splitString, ".");
		//
		////std::cout << "Found file: " << splitString.back() << std::endl;
		//
		//if (!std::strcmp(StringLowerCase(splitString.back()).c_str(), "obj"))
		//{
		//	filenames.push_back(allFiles[i]);
		//
		//	//std::cout << "Match " << filenames.size() << ": " << filenames.back() << std::endl;
		//}
	}

	//return;

	std::string input_command;

	std::string com_header = "D:\\_VV_COMPRESSION_SOFTWARES\\_DRACO\\draco-master\\_BUILD\\Release\\draco_decoder";

	std::string input_mesh;

	std::string output_number;

	//std::string output_tag = "DecompressMeshDRACO";

	//std::string output_extension = ".drc";

	std::string complete_output;

	for (int i = 0; i < filenames.size(); ++i)
	{
		input_mesh = filenames[i];

		output_number = GetNumberFixedLength(i, 6);

		complete_output = output_folder + "\\" + output_name + "_" + output_number + output_extension;

		input_command = com_header + " -i " + filenames[i] + " -o " + complete_output;

		//std::cout << input_command.c_str() << std::endl;

		//continue;

		system(input_command.c_str());
	}
}
