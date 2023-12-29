#pragma once

#include "TestSuite.h"

#include "DracoWrapper.h"

class DracoEXE_TestSuite : public TestSuite
{
	DracoWrapper dw;

	void PrecursorWrapperTest();
public:
	void run(int argc, char** argv);
};