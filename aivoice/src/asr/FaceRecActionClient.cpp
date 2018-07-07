#include "global.h"
#include "FaceRecActionClient.h"

FaceRecActionClient::FaceRecActionClient() : ac("face_rec_server", true)
{
    // ac.waitForServer();
}


void FaceRecActionClient::sendGoal(face_recGoal goal){
        ac.sendGoal(goal,
                boost::bind(&FaceRecActionClient::doneCb, this, _1, _2));
}

void FaceRecActionClient::doneCb(const actionlib::SimpleClientGoalState& state,
              const face_recResultConstPtr& result){
    for(int i=0;i<result->person.size();i++){
        std::string name = result->person.at(i);
        int confidence = result->confidence.at(i);
        string text="你好，";

        pub_tts(text+name,voiceName);
    }
}