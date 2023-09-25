#pragma once

#include <chrono>

class TestSuite
{
public:
	virtual void run(int argc, char** argv) = 0;
};