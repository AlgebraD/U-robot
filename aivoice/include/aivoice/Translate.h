#pragma once

#ifndef _TRANSLATE_
#define _TRANSLATE_

#include <iostream>
#include <time.h>
#include "cpr/cpr.h"

using namespace std;


struct BAIDU_TOKEN_INFO{
    string  token;      //调用百度翻译功能请求的token字符串
    time_t  expires;    //token的此时间前有效
};

class Translate
{

private:
	BAIDU_TOKEN_INFO bdti;

public:
	Translate();
	virtual ~Translate();

	string tran(string content,string srcLang,string destLang);
};

#endif