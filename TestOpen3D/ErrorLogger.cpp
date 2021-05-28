#include "ErrorLogger.h"
#include <iostream>
#include <assert.h>

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
	std::cout << error_message << GET_STACK() << std::endl;

	if (abort_on_error)
	{
		abort();
	}
}

