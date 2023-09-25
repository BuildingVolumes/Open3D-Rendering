#pragma once
#include <string>
#include <vector>
#include <iostream>

class AbstractCommand
{
	std::string name;
	std::string desc;

public:
	AbstractCommand(std::string name, std::string desc) : name(name), desc(desc) {}

	std::string GetDesc() { return desc; }
	std::string GetName() { return name; }

	int ExecuteCommand(std::vector<std::string> &specs, int currentSpec, int maxSpecs) {
		int argAmount = ArgumentAmount();

		if (maxSpecs < currentSpec + argAmount)
		{
			std::cout << "Invalid argument amount" << std::endl;

			return argAmount;
		}

		return ExecuteCommandProtected(specs, currentSpec, maxSpecs);
	}

protected:
	virtual int ExecuteCommandProtected(std::vector<std::string>& specs, int currentSpec, int maxSpecs) = 0;

	virtual int ArgumentAmount() = 0;
};