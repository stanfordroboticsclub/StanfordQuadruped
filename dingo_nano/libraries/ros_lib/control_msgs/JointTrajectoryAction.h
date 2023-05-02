#ifndef _ROS_control_msgs_JointTrajectoryAction_h
#define _ROS_control_msgs_JointTrajectoryAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "control_msgs/JointTrajectoryActionGoal.h"
#include "control_msgs/JointTrajectoryActionResult.h"
#include "control_msgs/JointTrajectoryActionFeedback.h"

namespace control_msgs
{

  class JointTrajectoryAction : public ros::Msg
  {
    public:
      typedef control_msgs::JointTrajectoryActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef control_msgs::JointTrajectoryActionResult _action_result_type;
      _action_result_type action_result;
      typedef control_msgs::JointTrajectoryActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    JointTrajectoryAction():
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

    virtual const char * getType() override { return "control_msgs/JointTrajectoryAction"; };
    virtual const char * getMD5() override { return "a04ba3ee8f6a2d0985a6aeaf23d9d7ad"; };

  };

}
#endif
