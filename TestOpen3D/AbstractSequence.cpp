#include "AbstractSequence.h"

#include "AdditionalUtilities.h"

AbstractSequence::AbstractSequence(std::string& root_folder)
{
	this->root_folder = root_folder;
}

std::vector<std::string> AbstractSequence::GetObjectsWithName(std::string& dataTag)
{
	std::vector<std::string> to_return;

	return to_return;
}

std::vector<std::string> AbstractSequence::GetObjectsWithNames(std::vector<std::string>& dataTags)
{
	std::vector<std::string> to_return;

	return to_return;
}
