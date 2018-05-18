#include "printf_console.h"
#include <stdio.h>
#include <iostream>


void logError(const char* msg, const char* arg0, const char* arg1, const char* arg2)
{
	std::cerr << msg << " " << arg0 << " " << arg1 << " " << arg2 << std::endl;
}
	
void logDebug(const char* msg, float v0, float v1)
{
	std::cerr << msg << " " << v0 << " " << v1 << std::endl;
};
void logDebug(const char* msg, const char* msg1, const char* arg1)
{
	std::cerr << msg << " " << msg1 << " " << arg1 << std::endl;
}

void logInform(const char* msg, const char* arg0)
{
	std::cerr << msg << " " << arg0 << std::endl;
}
void logWarn(const char* msg,int id, const char* arg0)
{
	std::cerr << msg << " " << id << " " << arg0 << std::endl;
}
