#pragma once

enum LOGGER_LEVEL
{
	LOG_TRACE,
	LOG_DEBUG,
	LOG_INFO,
	LOG_WARN,
	LOG_ERROR,
	LOG_FATAL
};

class CYYJSLogger
{
public:
	CYYJSLogger(void);
	~CYYJSLogger(void);

	/**
	*	写日志函数 
	*	@param	msg		日志内容
	*	@param	level	日志级别
	*/
	void log(char* msg, LOGGER_LEVEL level);
};

