#include "tello_actions/land_action.hpp"

LandActionClient::LandActionClient() : ac("tello_land", true) {
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
}

LandActionClient::~LandActionClient(){

}

void LandActionClient::takeOff() {
    tello_bridge::LandGoal goal;

    ac.sendGoal(goal,
                boost::bind(&LandActionClient::doneCb, this, _1, _2),
                boost::bind(&LandActionClient::activeCb, this),
                boost::bind(&LandActionClient::feedbackCb, this, _1)); 
}

void LandActionClient::doneCb(const actionlib::SimpleClientGoalState& state,
                const tello_bridge::LandResultConstPtr& result) {
    if(result->success)
        ROS_INFO("[RESULT] Land succesfull");
    ros::shutdown();
}

void LandActionClient::activeCb() {
  ROS_INFO("[INFO] Goal just went active");
}

void LandActionClient::feedbackCb(const tello_bridge::LandFeedbackConstPtr& feedback) {
  ROS_INFO("[FEEDBACK] Seconds taken so far: %i", feedback->seconds_taken);
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "land_action_client");
  LandActionClient land_client;
  land_client.takeOff();
  ros::spin();
  return 0;
}