#pragma once
#ifndef ASR_GLOBAL
#define ASR_GLOBAL

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "asr_ctrl_msg.h"
#include "asr_result_msg.h"
#include "nlp_msg.h"
#include "speak_notify_msg.h"
#include "translate_request_msg.h"
#include "tts_msg.h"
#include "ASR.h"
#include <unistd.h>
#include <iostream>
#include "PocketSphinxAsr.h"

using namespace ros;
using namespace std;

extern Publisher asr_pub;
extern Publisher tts_pub;
extern Publisher tran_pub;
extern Publisher nlp_pub;
extern Subscriber asr_ctrl_sub;
extern Subscriber speak_notify_sub;
extern ASR asr;
extern string voiceName;
extern bool robot_speaking;//机器人是否正在讲话
// extern Timer silence_check_timer; //

void pub_tts(string text,string voice);
void pub_asr(string text);
void pub_nlp(string text,string voice);
void pub_trans(string text,string srcLang,string destLang,string type);
#define silence_timeout 30 /*asr超时未识别到有效命令，自动进入静默状态，单位为秒*/
#endif
