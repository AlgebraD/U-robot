#include <ros/ros.h>
#include <std_msgs/String.h>


#include "texttosound.h"
#include "tts_msg.h"
#include "speak_msg.h"

using namespace ros;

Publisher p;
Subscriber sub;

void ttsCallback(const aivoice::tts_msgConstPtr& msg){//(const std_msgs::String::ConstPtr& msg){

  char buf[1024];
  sprintf(buf,"recv tts msg:\n \
           rdn:%d \n \
           volume:%d \n \
           pitch:%d \n \
           speed:%d \n \
           sample_rate:%d \n \
           voice_name:%s \n \
           text_encoding:%s \n \
           text:%s \n", msg->rdn,msg->volume,msg->pitch,msg->speed,msg->sample_rate,
         ((string)msg->voice_name).data() ,
          ((string)msg->text_encoding).data() ,
          ((string)msg->text).data() );
  ROS_INFO("%s",buf);

  TextToSound tts;
  TTS_TYPE ttsType = (TTS_TYPE)msg->tts_type;
  tts.setTypeOfTTS(ttsType);
  tts.init();

  TTS_PARAM param;
  ZERO_TTS_PARAM(param);
  param.pitch   = msg->pitch;
  param.rdn     = msg->rdn;
  param.sample_rate = msg->sample_rate;
  param.speed   = msg->speed;
  param.text_encoding   = ((string)msg->text_encoding).length() > 0 ? msg->text_encoding : "";
  param.voice_name  = ((string)msg->voice_name).length() > 0 ? msg->voice_name : "";
  param.volume  = msg->volume;

  string text = ((string)msg->text).data();
  string audioFilePath = tts.tts(text,param);//调用tts生成语音文件

  if(audioFilePath.length() > 0){
    
    aivoice::speak_msg msg;
    msg.audioFilePath = audioFilePath;
    msg.order = 0;
    if(ttsType == TTS_BAIDU)
      msg.codecFormat = "mp3";
    else
      msg.codecFormat = "wav";
    p.publish(msg); //向说话节点发消息，播放语音文件
  }else{
    ROS_WARN("%s","tts返回语音文件路径为空！");
  }
  tts.uninit();

}

int main(int argc,char** argv){

    init(argc,argv,"tts_node");
    NodeHandle n;
    sub = n.subscribe("tts_topic",1,ttsCallback);
    p = n.advertise<aivoice::speak_msg>("speak_topic",10);
    spin();

    return 0;
}


