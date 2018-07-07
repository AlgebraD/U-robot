#include "global.h"

#include "FaceRecActionClient.h"



#define	BUFFER_SIZE	4096

FaceRecActionClient *faceRecClient = NULL;

Publisher asr_pub;
Publisher tts_pub;
Publisher tran_pub;
Publisher nlp_pub;
Subscriber asr_ctrl_sub;
Subscriber speak_notify_sub;
ASR asr;
bool robot_speaking = false;

PocketSphinxAsr psAsr("/usr/local/share/pocketsphinx/model/zh/hmm",
                        "/usr/local/share/pocketsphinx/model/zh/1271.lm",
                        "/usr/local/share/pocketsphinx/model/zh/1271.dic");
// PocketSphinxAsr psAsr("/usr/local/share/pocketsphinx/model/en-us/en-us",
//                         "/usr/local/share/pocketsphinx/model/en-us/en-us.lm.bin",
//                         "/usr/local/share/pocketsphinx/model/en-us/cmudict-en-us.dict");
// PocketSphinxAsr psAsr("/usr/local/src/pocketsphinx-5prealpha/model/cn/zh_broadcastnews_ptm256_8000",
//                         "/usr/local/src/pocketsphinx-5prealpha/model/cn/TAR1271/1271.lm",
//                         "/usr/local/src/pocketsphinx-5prealpha/model/cn/TAR1271/1271.dic");

string voiceName = "xiaokun";   //当前的发言人
string lastVoiceName = voiceName;
bool    ignored  = false;    //是否忽略识别出的结果
// bool    isWakeUp = false;   //是否已唤醒，未唤醒状态使用离线语音识别，唤醒状态使用在线识别
bool    waitToAsr   = false;//是否正等待消息以开始asr，发布asr结果消息时置真，收到开始放音消息时，置假
time_t  beginDetectSilenceTime = 0; //开始检测静默状态的时间 

// Timer   silence_check_timer;

/****************讯飞asr回调函数*****************/
static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;

bool handleAsrResult(char *text);
bool containWords(char* str1,const char** wordArray,int len);
bool containWordsInOrder(char* str1,const char** wordArray,int len);

void* closeFlyAsrThread(void* arg);//关闭讯飞识别线程的线程，因为识别结果回调函数是在asr线程内部调用的，所以要另外开一个线程关闭asr线程

struct TRANSLATE_INFO{
    bool isTranslate;
    string srcLanguage;
    string destLanguage;
}tran_info;

enum ASR_STATE{
    asr_close,        //关闭状态，需要外部命令开启语音识别
    asr_offline,    //静默状态，离线语音识别等待命令词唤醒
    asr_online,     //已唤醒状态，切换到在线识别
    asr_chat,       //在线对话模式
}asr_state;



//讯线语音识别结果回调函数 
void on_result(const char *result, char is_last)
{
	if (result) {
		size_t left = g_buffersize - 1 - strlen(g_result);
		size_t size = strlen(result);
		if (left < size) {
			g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
			if (g_result)
				g_buffersize += BUFFER_SIZE;
			else {
				printf("mem alloc failed\n");
				return;
			}
		}
		strncat(g_result, result, size);

        if(is_last){

            // ignored = false;
            if(robot_speaking)
                return;

            //发布识别结果消息
            if(g_result != NULL && strlen(g_result) > 0)
                pub_asr(g_result);
        }
	}
}

void on_speech_begin()
{
	if (g_result)
	{
		free(g_result);
	}
	g_result = (char*)malloc(BUFFER_SIZE);
	g_buffersize = BUFFER_SIZE;
	memset(g_result, 0, g_buffersize);

	printf("Start Listening...\n");
    // system("play start_record.wav");
    // sleep(1);
}

void on_speech_end(int reason)
{
	if (reason == END_REASON_VAD_DETECT){
		printf("\nSpeaking done \n");
		// //结束上一段asr，开始新的asr
        asr.stopAsr();
        // //  system("play stop_record.wav");
         sleep(0.5);
        // if((strlen(g_result) <= 0 || ignored) && robot_speaking == false)
        asr.startAsr();
        // else
        //     waitToAsr = true;
	}else{
		printf("\nRecognizer error %d\n", reason);
    }
}

//pocketsphinx 语音识别结果回调函数 
void psAsrCallback(string result){
    ROS_DEBUG("asr识别结果：%s",result.c_str());
    // cout<<"asr识别结果："<<result.c_str()<<endl;
    if(result.size() > 0)
        pub_asr(result);

}


/*****************************/
void speakNotifyCallback(const aivoice::speak_notify_msgConstPtr& msg){
    char buf[512];
    sprintf(buf,"recv speak_notify_msg:\n \
           speak_state:%d", msg->speak_state);
     ROS_INFO("%s",buf);


    int speak_state = msg->speak_state;
    if(speak_state == 0 ){//放音开始通知
        ROS_DEBUG("recv speaking start");
        sleep(1);
        robot_speaking = true;
        if(asr_state == asr_online){
            if(asr.isAsring ){
                asr.stopAsr();
            }
        }else if(asr_state == asr_offline){
            psAsr.stopListen();
        }
        
    }else if(speak_state == 1 ){//放音结束通知
        ROS_DEBUG("recv speaking stop");
        
        robot_speaking = false;
        waitToAsr = false;
        if(asr_state == asr_online){
            if(!asr.isAsring ){
                asr.startAsr();
            }
        }else if(asr_state == asr_offline){
            psAsr.beginListen(psAsrCallback);
        }
    }
}

void asrCtrlCallback(const aivoice::asr_ctrl_msgConstPtr& msg){//(const std_msgs::String::ConstPtr& msg){

  char buf[512];
  sprintf(buf,"recv asr contrl msg:\n \
           order:%d \
					 asrType:%d", msg->order,msg->asrType);
  ROS_DEBUG("%s",buf);

  int order     = msg->order;
  int asrType   = msg->asrType;

  if(order == 0){//开始asr

    if(asr_state != asr_close)
        return;
    else{
        psAsr.beginListen(psAsrCallback);
    }
    asr_state = asr_offline;

  }else if(order == 1){//停止asr

    if(asr_state == asr_close)
        return;
    else if(asr_state == asr_offline){//停止离线识别
        psAsr.stopListen(); 
    }else if(asr_state == asr_online){//停止在线识别
        if(asrType == ASR_TYPE_XUNFEI){
            if(asr.uninit() == 1){
                ROS_INFO("%s","Stop xunfei ASR success!");
            }else{
                ROS_WARN("%s","Stop xunfei ASR Failed!");
            }
        }
    }
    asr_state = asr_close;

  }else if(order == 2){//切换到离线识别
    if(asr_state == asr_offline)
        return;
    else if(asr_state > asr_offline){//先停掉在线识别
        if(asrType == ASR_TYPE_XUNFEI){
            if(asr.uninit() == 1){
                ROS_INFO("%s","Stop xunfei ASR success!");
            }else{
                ROS_WARN("%s","Stop xunfei ASR Failed!");
            }
        }
    }

    psAsr.beginListen(psAsrCallback);
    asr_state = asr_offline;

  }else if(order == 3){//切换到在线识别

    if(asr_state == asr_online)
        return;
    else if(asr_state == asr_offline){//先停掉离线识别
        psAsr.stopListen();
    }

    if(asrType == ASR_TYPE_XUNFEI){

        speech_rec_notifier notifier = {on_result, on_speech_begin, on_speech_end };
        int result = 0;
        if( (result =asr.init_xunfei(notifier)) != 1){
             ROS_WARN("init_xunfei return %d",result);
        }else{
            if( (result =asr.startAsr()) != 1){
                ROS_WARN("start xunfei Asr return %d",result);
            }else
                ROS_INFO("%s","Start xunfei ASR success!");
        }
    }
    asr_state = asr_online;

  }


}


void* closeFlyAsrThread(void* arg){
    asr_state = asr_offline;
    asr.uninit();  //关闭在线识别
    psAsr.beginListen(psAsrCallback);   //开启离线识别
    beginDetectSilenceTime = 0;
    return NULL;
}

//在等待tts放音时，asr线程是关闭的，如果tts放音失败，asr就再也不能启动。该线程确保tts消息超时后，asr自动激活
void* guaranteeAsrWork(void* arg){
    time_t beginTime = 0;
    int timeOut = 30;
    while(1){
        if(waitToAsr){
            if(beginTime == 0)
                beginTime = time(NULL);
            if(time(NULL)-beginTime > timeOut){
                robot_speaking = false;
                waitToAsr = false;
                beginTime = 0;
                ROS_WARN("%s","recv speakinng msg timeout ,restart asr");
                if(asr_state == asr_online){
                    if(!asr.isAsring ){
                        asr.startAsr();
                    }
                }else if(asr_state == asr_offline){
                    psAsr.beginListen(psAsrCallback);
                }
            }
        }else{
            beginTime = 0;
        }

        if( beginDetectSilenceTime > 0 
            && time(NULL)-beginDetectSilenceTime > silence_timeout
            && asr_state != asr_offline){
                pthread_t tid;
            pthread_create(&tid,NULL,closeFlyAsrThread,NULL);
            pub_tts("静默",voiceName);
        }

        usleep(10000);
    }
    return NULL;
}

void pub_tts(string text,string voice){
    aivoice::tts_msg msg;
	msg.tts_type = 1;
  	msg.voice_name = voice;
    msg.text = text;
  	msg.text_encoding = "utf8";
    tts_pub.publish(msg); 
	ROS_DEBUG("发送tts %s",text);
}

void pub_asr(string text){
    ROS_DEBUG("asr result：%s",text);
    aivoice::asr_result_msg msg;
    msg.text = text;
    // msg.voice = voice;
    asr_pub.publish(msg);
}

void pub_nlp(string text,string voice){
    ROS_DEBUG("nlp request：%s",text);
    aivoice::nlp_msg msg;
    msg.text = text;
    msg.voice = voice;
    nlp_pub.publish(msg);
}

void pub_trans(string text,string srcLang,string destLang,string type){
    ROS_DEBUG("发送translate_request_msg:%s",text);
    aivoice::translate_request_msg msg;
    msg.srcText = text;
    msg.srcLanguage = srcLang;
    msg.destLanguage = destLang;
    msg.type = type;
    tran_pub.publish(msg);
}

int main(int argc,char** argv){

    //启动后，开启离线识别，等待命令词唤醒
    if(psAsr.beginListen(psAsrCallback))
        asr_state = asr_offline;
    else{
        asr_state = asr_close; 
        ROS_ERROR("%s","离线识别启动失败");
    }

    //开启asr重启线程
    pthread_t tid;
    pthread_create(&tid,NULL,guaranteeAsrWork,NULL);

    init(argc,argv,"asr_node");
    NodeHandle n;
    asr_ctrl_sub = n.subscribe("asr_ctrl_topic",10,asrCtrlCallback); 
    speak_notify_sub = n.subscribe("speak_notify_topic",10,speakNotifyCallback);
    asr_pub = n.advertise<aivoice::asr_result_msg>("asr_result_topic",10);
    nlp_pub = n.advertise<aivoice::nlp_msg>("nlp_topic",10);
    tts_pub = n.advertise<aivoice::tts_msg>("tts_topic",10);
    tran_pub = n.advertise<aivoice::translate_request_msg>("translate_topic",10);

    faceRecClient = new FaceRecActionClient();

    // silence_check_timer = n.createTimer(ros::Duration(0.1), timerCallback);
    spin();
    


    return 0;
}