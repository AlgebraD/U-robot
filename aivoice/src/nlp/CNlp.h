#pragma once
#ifndef _CNlp_
#define _CNlp_

#include <curl/curl.h>
#include <time.h>
#include <sstream>

using namespace std;

enum NLP_TYPE{
	NLP_TYPE_TULING	//图灵
};

class CNlp
{

private:
	CURL        *curl;              //使用curl实现http请求
	string      httpResponseText;   //使用curl请求http的返回文本
	NLP_TYPE	nlpType;			//使用哪种NLP功能
	long		userId;				//用户id，同一个id共享同一个图灵的上下文

private:
	int init();
	int uninit();
	string processUseTuling(string text);//使用图灵的自然语言处理功能

public:
	CNlp();
	virtual ~CNlp();
	
	/**
	调用第三方平台功能执行自然语言处理，并返回处理结果，
	text	[IN]	输入的自然语言
	返回	成功：应答文本；失败：空字符串
	*/
	string process(string text);
};

#endif