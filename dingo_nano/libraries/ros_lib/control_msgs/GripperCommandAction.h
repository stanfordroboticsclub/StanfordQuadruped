#ifndef _ROS_control_msgs_GripperCommandAction_h
#define _ROS_control_msgs_GripperCommandAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "control_msgs/GripperCommandActionGoal.h"
#include "control_msgs/GripperCommandActionResult.h"
#include "control_msgs/GripperCommandActionFeedback.h"

namespace control_msgs
{

  class GripperCommandAction : public ros::Msg
  {
    public:
      typedef control_msgs::GripperCommandActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef control_msgs::GripperCommandActionResult _action_result_type;
      _action_result_type action_result;
      typedef control_msgs::GripperCommandActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    GripperCommandAction():
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

    virtual const char * getType() override { return "control_msgs/GripperCommandAction"; };
    virtual const char * getMD5() override { return "950b2a6ebe831f5d4f4ceaba3d8be01e"; };

  };

}
#endif
