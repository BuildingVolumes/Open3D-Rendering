#pragma once

#include "TestSuite.h"
#include <string>
#include <vector>

class AlembicExportSuite : public TestSuite
{
	std::string root, tex_handle, mesh_handle;

	std::vector<std::string> mesh_files;
	std::vector<std::string> texture_files;

	float* DoubleArrayToFloatArray(double* elems, size_t count);

public:
	AlembicExportSuite();

	void run(int argc, char** argv);

	void SynthesizeAlembic(std::string root_folder, std::string texture_names, std::string mesh_names);
};