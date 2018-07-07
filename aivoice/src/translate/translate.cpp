#include "ros/ros.h"
#include "std_msgs/String.h"
#include "translate_request_msg.h"
#include "tts_msg.h"
#include <sstream>
#include <iostream>
#include "Translate.h"
#include "trans.h"
#include <ros/console.h>

using namespace ros;
using namespace std;

Publisher tts_pub;
Translate translate;

void translateCallback(const aivoice::translate_request_msgConstPtr& msg){

	string srcText 		= 		((string)msg->srcText).data();
	string srcLanguage 	= 		((string)msg->srcLanguage).data();
	string destLanguage = 		((string)msg->destLanguage).data();
	string type 		= 		((string)msg->type).data();

	char buf[1024];
	sprintf(buf,"recv translate_request_msg msg:\n \
			srcText:%s \n \
			srcLanguage:%s \n \
			destLanguage:%s \n \
			type:%s \n",srcText.data(),srcLanguage.data(),destLanguage.data(),type.data());
	ROS_INFO("%s",buf);

	string result = translate.tran(srcText,srcLanguage,destLanguage);
	if(result.length() > 0){
		aivoice::tts_msg msg;
		msg.tts_type = 1;
		if(destLanguage.compare("en") == 0)
  			msg.voice_name = "catherine";
		else
			msg.voice_name = "jinger";
    	msg.text = result;
  		msg.text_encoding = "utf8";
    	tts_pub.publish(msg); 
		ROS_DEBUG("%s","发送tts");
	}
}

bool transServFunc(aivoice::trans::Request  &req,  
         aivoice::trans::Response &res)  
{  
  ROS_DEBUG("%s","transServFunc: text:"+req.src_text+" srcLan:"+req.src_language+" destLan:"+req.dest_language);
  string result = translate.tran(req.src_text,req.src_language,req.dest_language);
  res.result_text = result;
  return true;
}

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "translate_node");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	ros::NodeHandle n;
	tts_pub = n.advertise<aivoice::tts_msg>("tts_topic", 10);
    Subscriber sub = n.subscribe("translate_topic",10,translateCallback);
	ServiceServer srv = n.advertiseService("translate_service", transServFunc); 
    spin();
	return 0;
}