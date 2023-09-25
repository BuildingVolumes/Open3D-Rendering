#pragma once

#include "TestSuite.h"
#include "VolumeSequence.h"

class TestMultiMesh : public TestSuite
{
	VolumeSequence vs;

public:
	void run(int argc, char** argv);

	void SaveGridFourier();

	void SaveGridZip();

	void LoadGridFourier();

	void SaveMultipleGridsFourier();

	void SaveGridZipIframes();

	void SaveGridFourierIframes();
};