#pragma once

#include <string>
#include <vector>

class ErrorLogger {
	class StackMessage {
		std::string my_name;

		friend class ErrorLogger;
	public:
		StackMessage(std::string name);
		~StackMessage();
	};

	static std::vector<StackMessage*> call_stack;

	friend class StackMessage;
public:
	/// <summary>
	/// The current call stack
	/// </summary>
	/// <returns>The call stack</returns>
	static std::string GET_STACK();

	/// <summary>
	/// Use to inform that an error has occured, and retrieve the call stack
	/// </summary>
	/// <param name="error_message">Message to display</param>
	/// <param name="assert_on_error">Forcefully halt the program once this error has occured</param>
	static void LOG_ERROR(std::string error_message, bool abort_on_error = false);

	/// <summary>
	/// Executes an object's function for it, enabling a call stack to form
	/// </summary>
	/// <typeparam name="N">Object type</typeparam>
	/// <typeparam name="T">Return type</typeparam>
	/// <typeparam name="...Args">Function arguments</typeparam>
	/// <param name="call_msg">Message to log on the stack</param>
	/// <param name="obj">Object to call function from</param>
	/// <param name="FP">Function pointer</param>
	/// <param name="...realArgs">Function arguments</param>
	/// <returns>The object returned from the function call</returns>
	template<class N, class T, typename ...Args>
	static T EXECUTE(std::string call_msg, N* obj, T(N::* FP)(Args &...args), Args &...realArgs)
	{
		StackMessage sm(call_msg);
		return (obj->*FP)(realArgs...);
	}

	/// <summary>
	/// Executes an object's function for it, enabling a call stack to form
	/// </summary>
	/// <typeparam name="N">Object type</typeparam>
	/// <typeparam name="T">Return type</typeparam>
	/// <typeparam name="...Args">Function arguments</typeparam>
	/// <param name="call_msg">Message to log on the stack</param>
	/// <param name="obj">Object to call function from</param>
	/// <param name="FP">Function pointer</param>
	/// <param name="...realArgs">Function arguments</param>
	/// <returns>The object returned from the function call</returns>
	template<class N, class T, typename ...Args>
	static T EXECUTE(std::string call_msg, N* obj, T(N::* FP)(Args ...args), Args ...realArgs)
	{
		StackMessage sm(call_msg);
		return (obj->*FP)(realArgs...);
	}

	/// <summary>
	/// Executes a function into a managed call stack
	/// </summary>
	/// <typeparam name="T">Return type</typeparam>
	/// <typeparam name="...Args">Function arguments</typeparam>
	/// <param name="call_msg">Message to log on the stack</param>
	/// <param name="FP">Function pointer</param>
	/// <param name="...realArgs">Function arguments</param>
	/// <returns>The object returned from the function call</returns>
	template<class T, typename ...Args>
	static T EXECUTE(std::string call_msg, T(*FP)(Args &...args), Args &...realArgs)
	{
		StackMessage sm(call_msg);
		return (*FP)(realArgs...);
	}

	/// <summary>
	/// Executes a function into a managed call stack
	/// </summary>
	/// <typeparam name="T">Return type</typeparam>
	/// <typeparam name="...Args">Function arguments</typeparam>
	/// <param name="call_msg">Message to log on the stack</param>
	/// <param name="FP">Function pointer</param>
	/// <param name="...realArgs">Function arguments</param>
	/// <returns>The object returned from the function call</returns>
	template<class T, typename ...Args>
	static T EXECUTE(std::string call_msg, T(*FP)(Args ...args), Args ...realArgs)
	{
		StackMessage sm(call_msg);
		return (*FP)(realArgs...);
	}
};

#define E_LOG(msg, make_abort) ErrorLogger::LOG_ERROR(std::string(msg) + ", " + std::string(__func__) + ", " + std::to_string(__LINE__), make_abort);
