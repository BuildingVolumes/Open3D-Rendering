#include "ErrorLogger.h"
#include <iostream>
#include <assert.h>
#include <fstream>

std::vector<ErrorLogger::StackMessage*> ErrorLogger::call_stack = std::vector<ErrorLogger::StackMessage*>();

ErrorLogger::StackMessage::StackMessage(std::string name)
{
	my_name = name;
	ErrorLogger::call_stack.push_back(this);
}

ErrorLogger::StackMessage::~StackMessage()
{
	ErrorLogger::call_stack.pop_back();
}

std::string ErrorLogger::GET_STACK()
{
	std::string to_return = "";

	for (int i = 0; i < call_stack.size(); ++i)
	{
		to_return += "\n>\t";
		to_return += call_stack[i]->my_name;
	}

	return to_return;
}

void ErrorLogger::LOG_ERROR(std::string error_message, bool abort_on_error)
{
	std::string complete_error = error_message + GET_STACK();

	std::cout << complete_error << std::endl;

	std::ofstream writer;

	writer.open("Open3DErrorList.txt");

	writer.write(complete_error.c_str(), complete_error.length());

	writer.close();

	if (abort_on_error)
	{
		abort();
	}
}

