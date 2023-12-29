#pragma once

#include "TestSuite.h"
#include "VolumeSequence.h"

class SingleMeshSuite : public TestSuite
{
	VolumeSequence vs;

public:
	void run(int argc, char** argv);
};