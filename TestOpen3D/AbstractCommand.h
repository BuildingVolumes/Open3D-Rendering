#pragma once
#include <string>

class AbstractCommand
{
	std::string name;
	std::string desc;

public:
	AbstractCommand(std::string name, std::string desc) : name(name), desc(desc) {}

	std::string GetDesc() { return desc; }
	std::string GetName() { return name; }

	virtual int ExecuteCommand(char** specs, int currentSpec, int maxSpecs) = 0;
};