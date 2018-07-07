#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nlp_msg.h"
#include "tts_msg.h"
#include "nlp.h"
#include "CNlp.h"
#include "tts/texttosound.h"
#include <sstream>
#include <ros/console.h>

using namespace std;
using namespace ros;

ros::Publisher tts_pub; 
CNlp nlp;

void asrResultCallback(const aivoice::nlp_msgConstPtr& msg){//(const std_msgs::String::ConstPtr& msg){

  char buf[512];
  sprintf(buf,"recv nlp_msg:\n \
           text:%s",((string)msg->text).data());
  ROS_INFO("%s",buf);

  string text = (string)msg->text;
  string voice = (string)msg->voice;
  if(text.length() <= 0)
  	return;

  string result = nlp.process(text);

  if(result.length() > 0){
	aivoice::tts_msg msg;
	msg.tts_type = TTS_XUNFEI;
  	msg.voice_name = voice.length() == 0 ? "jinger" : voice;
  	msg.text = result;
  	msg.text_encoding = "utf8";
    tts_pub.publish(msg); 
	ROS_DEBUG("%s","发送tts");
  }

}

bool nlpServFunc(aivoice::nlp::Request  &req,  
         aivoice::nlp::Response &res)  
{  
  ROS_DEBUG("%s","nlpServFunc:"+req.src_text);
  string text = req.src_text;
  string result = nlp.process(text);
  res.result_text = result;
  return true;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[])
{

	ros::init(argc, argv, "nlp_node");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<aivoice::nlp_msg>("nlp_topic", 1,asrResultCallback);
	tts_pub = n.advertise<aivoice::tts_msg>("tts_topic", 10);
	ros::ServiceServer nlpServ = n.advertiseService("nlp_service", nlpServFunc);   
	ros::spin();
	return 0;
}