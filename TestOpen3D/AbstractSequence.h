#pragma once

#include <string>
#include <vector>

class AbstractSequence
{
	std::string root_folder;
public:
	AbstractSequence(std::string &root_folder);

	std::vector<std::string> GetObjectsWithName(std::string& dataTag);
	std::vector<std::string> GetObjectsWithNames(std::vector<std::string> &dataTags);
};