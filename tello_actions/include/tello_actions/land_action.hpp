#ifndef LAND_ACTION_H_
#define LAND_ACTION_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <tello_bridge/LandAction.h>
#include <tello_bridge/LandGoal.h>
#include <tello_bridge/LandResult.h>
#include <tello_bridge/LandFeedback.h>

class LandActionClient {
    private:
        actionlib::SimpleActionClient<tello_bridge::LandAction> ac;

    public:
        LandActionClient();
        ~LandActionClient();

        void takeOff();

        void doneCb(const actionlib::SimpleClientGoalState& state,
                    const tello_bridge::LandResultConstPtr& result);

        void activeCb();

        void feedbackCb(const tello_bridge::LandFeedbackConstPtr& feedback);

};

#endif // LAND_ACTION_H_