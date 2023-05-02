#ifndef _ROS_actionlib_TwoIntsAction_h
#define _ROS_actionlib_TwoIntsAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "actionlib/TwoIntsActionGoal.h"
#include "actionlib/TwoIntsActionResult.h"
#include "actionlib/TwoIntsActionFeedback.h"

namespace actionlib
{

  class TwoIntsAction : public ros::Msg
  {
    public:
      typedef actionlib::TwoIntsActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef actionlib::TwoIntsActionResult _action_result_type;
      _action_result_type action_result;
      typedef actionlib::TwoIntsActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    TwoIntsAction():
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

    virtual const char * getType() override { return "actionlib/TwoIntsAction"; };
    virtual const char * getMD5() override { return "6d1aa538c4bd6183a2dfb7fcac41ee50"; };

  };

}
#endif
