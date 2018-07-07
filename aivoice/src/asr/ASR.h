#pragma once
#ifndef _ASR_
#define _ASR_

#include "speech_recognizer.h"

enum ASR_TYPE{
		ASR_TYPE_BAIDU,	//百度
		ASR_TYPE_XUNFEI	//迅飞
};

class ASR
{
	//asr的类型

private:
	ASR_TYPE asrType;
	speech_rec	iat;


public:
	speech_rec_notifier asr_notifer;	//讯飞asr功能回调函数地址结构
	bool		isAsring;//是否正在asr
	bool 		isInited;//是否已经初始化
	
public:
	ASR();
	virtual ~ASR();

	/*
	讯飞asr初始化
	asr_notifer	[IN]	asr功能回调函数

	*/
	int init_xunfei(speech_rec_notifier asr_notifer);
	int uninit();

	//启动asr
	int startAsr();

	//停止asr
	int stopAsr();
};

#endif
