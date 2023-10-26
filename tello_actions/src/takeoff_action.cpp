#include "tello_actions/takeoff_action.hpp"

TakeoffActionClient::TakeoffActionClient() : ac("tello_takeoff", true) {
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
}

TakeoffActionClient::~TakeoffActionClient(){

}

void TakeoffActionClient::takeOff() {
    tello_bridge::TakeoffGoal goal;

    ac.sendGoal(goal,
                boost::bind(&TakeoffActionClient::doneCb, this, _1, _2),
                boost::bind(&TakeoffActionClient::activeCb, this),
                boost::bind(&TakeoffActionClient::feedbackCb, this, _1)); 
}

void TakeoffActionClient::doneCb(const actionlib::SimpleClientGoalState& state,
                const tello_bridge::TakeoffResultConstPtr& result) {
    if(result->success)
        ROS_INFO("[RESULT] TakeOff succesfull");
    ros::shutdown();
}

void TakeoffActionClient::activeCb() {
  ROS_INFO("[INFO] Goal just went active");
}

void TakeoffActionClient::feedbackCb(const tello_bridge::TakeoffFeedbackConstPtr& feedback) {
  ROS_INFO("[FEEDBACK] Seconds taken so far: %i", feedback->seconds_taken);
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "takeoff_action_client");
  TakeoffActionClient takeoff_client;
  takeoff_client.takeOff();
  ros::spin();
  return 0;
}