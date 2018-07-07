#pragma once
#ifndef POCKET_SPHINX_ASR 
#define POCKET_SPHINX_ASR

#include <iostream>
#include <pocketsphinx.h>
#include <sphinxbase/ad.h>
#include <pthread.h>
 #include <unistd.h>

using namespace std;

typedef void(*psAsrResultFunc)(string) ;

class PocketSphinxAsr
{

private:
	pthread_t	tid;//asr线程id
	

public:
	//pocketsphinx使用变量 
	ps_decoder_t 	*ps;		
 	cmd_ln_t 		*config;
	bool			stopThread;//控制线程结束标志	
	bool			isListening;//是否正在离线识别
	psAsrResultFunc	asrCallbackFunc;//asr结果回调函数
	
public:
	PocketSphinxAsr(string hmmDir,string lmPath,string dictPath);
	virtual ~PocketSphinxAsr();

	/**
	开始离线语音识别
	asrResultFunc	:	离线语音误别结果
	*/
	bool beginListen(psAsrResultFunc resultCallback);

	/**
	停止监听
	*/
	void stopListen();

	// /**
	// asr线程识别出结果后，调用该函数
	// asrStr: asr识别结果
	// */
	// void getAsrResult(string asrStr);

private:
	void init(string hmmDir,string lmPath,string dictPath);
	

};

#endif