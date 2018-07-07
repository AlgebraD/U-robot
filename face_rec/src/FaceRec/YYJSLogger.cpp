#include "YYJSLogger.h"
#include <iostream>

using namespace std;

CYYJSLogger::CYYJSLogger(void)
{
}


CYYJSLogger::~CYYJSLogger(void)
{
}

void CYYJSLogger::log(char* msg, LOGGER_LEVEL level){
	cout<<msg<<endl;
}