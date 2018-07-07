#ifndef FACE_REC_ACTION_CLIENT
#define FACE_REC_ACTION_CLIENT

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <face_rec/face_recAction.h>

using namespace face_rec;
typedef actionlib::SimpleActionClient<face_recAction> FaceRecActionlibClient;

class FaceRecActionClient
{
public:
    FaceRecActionClient();
    void sendGoal(face_recGoal goal);
    void doneCb(const actionlib::SimpleClientGoalState& state,
              const face_recResultConstPtr& result);

//   MyNode() : ac("face_rec_server", true)
//   {
//     ROS_INFO("Waiting for action server to start.");
//     ac.waitForServer();
//     ROS_INFO("Action server started, sending goal.");
//   }

//   void doStuff(int order)
//   {
//     FibonacciGoal goal;
//     goal.order = order;

//     // Need boost::bind to pass in the 'this' pointer
//     ac.sendGoal(goal,
//                 boost::bind(&MyNode::doneCb, this, _1, _2),
//                 Client::SimpleActiveCallback(),
//                 Client::SimpleFeedbackCallback());

//   }

//   void doneCb(const actionlib::SimpleClientGoalState& state,
//               const FibonacciResultConstPtr& result)
//   {
//     ROS_INFO("Finished in state [%s]", state.toString().c_str());
//     ROS_INFO("Answer: %i", result->sequence.back());
//     ros::shutdown();
//   }

private:
  FaceRecActionlibClient ac;
};

#endif