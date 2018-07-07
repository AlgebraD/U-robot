#include <ros/ros.h>
#include <std_msgs/String.h>
#include "speak_notify_msg.h"
#include "speak_msg.h"

using namespace ros;
using namespace std;

bool canSpeak = true;
Publisher pub;

void sayCallback(const aivoice::speak_msgConstPtr& msg){//(const std_msgs::String::ConstPtr& msg){

  char buf[512];
  sprintf(buf,"recv speak msg:\n \
           order:%d \n \
           audioFilePath:%s \n \
           codecFormat:%s", msg->order,
          ((string)msg->audioFilePath).data(),
          ((string)msg->codecFormat).data());
  ROS_INFO("%s",buf);

    string audioFilePath =  (string)msg->audioFilePath;
    int order = msg->order;

    string cmd;
    int status = 0;
    if(order == 0 ){
        if(!canSpeak)
            return;

        //发送开始话音通知
        aivoice::speak_notify_msg notify;
        notify.speak_state = 0;
        pub.publish(notify);

        if(msg->codecFormat.compare("mp3") == 0){
            cmd = "play -tmp3 "+audioFilePath;
                status = system(cmd.data());
        }else if( msg->codecFormat.compare("wav") == 0){
            cmd = "play "+audioFilePath;
                status = system(cmd.data());
        }

        Duration(1).sleep();
        //发送结束放音通知
        notify.speak_state = 1;
        pub.publish(notify);

        cmd = "rm "+audioFilePath;
        status = system(cmd.data());

    }else if(order == 1){//禁言
        canSpeak = false;
    }else if(order == 2){//发言
        canSpeak = true;
    }
    
}

int main(int argc,char** argv){

    ros::init(argc,argv,"speak_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("speak_topic",10,sayCallback);
    pub = n.advertise<aivoice::speak_notify_msg>("speak_notify_topic", 10);
    spin();

    return 0;
}


