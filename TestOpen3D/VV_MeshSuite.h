#pragma once

#include "TestSuite.h"

#include "VV_Mesh.h"

class VV_MeshSuite : public TestSuite
{
	VV_Mesh test_mesh;

	void TestLoadingAndSaving();
	void TestDecimation();

public:
	void run(int argc, char** argv);
};