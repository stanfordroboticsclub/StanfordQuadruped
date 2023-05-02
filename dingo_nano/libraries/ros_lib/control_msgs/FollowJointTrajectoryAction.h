#ifndef _ROS_control_msgs_FollowJointTrajectoryAction_h
#define _ROS_control_msgs_FollowJointTrajectoryAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "control_msgs/FollowJointTrajectoryActionResult.h"
#include "control_msgs/FollowJointTrajectoryActionFeedback.h"

namespace control_msgs
{

  class FollowJointTrajectoryAction : public ros::Msg
  {
    public:
      typedef control_msgs::FollowJointTrajectoryActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef control_msgs::FollowJointTrajectoryActionResult _action_result_type;
      _action_result_type action_result;
      typedef control_msgs::FollowJointTrajectoryActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    FollowJointTrajectoryAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "control_msgs/FollowJointTrajectoryAction"; };
    virtual const char * getMD5() override { return "bc4f9b743838566551c0390c65f1a248"; };

  };

}
#endif
