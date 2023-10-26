#ifndef TAKEOFF_ACTION_H_
#define TAKEOFF_ACTION_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <tello_bridge/TakeoffAction.h>
#include <tello_bridge/TakeoffGoal.h>
#include <tello_bridge/TakeoffResult.h>
#include <tello_bridge/TakeoffFeedback.h>

class TakeoffActionClient {
    private:
        actionlib::SimpleActionClient<tello_bridge::TakeoffAction> ac;

    public:
        TakeoffActionClient();
        ~TakeoffActionClient();

        void takeOff();

        void doneCb(const actionlib::SimpleClientGoalState& state,
                    const tello_bridge::TakeoffResultConstPtr& result);

        void activeCb();

        void feedbackCb(const tello_bridge::TakeoffFeedbackConstPtr& feedback);

};

#endif // TAKEOFF_ACTION_H_