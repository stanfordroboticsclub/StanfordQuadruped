#ifndef _ROS_actionlib_tutorials_FibonacciAction_h
#define _ROS_actionlib_tutorials_FibonacciAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "actionlib_tutorials/FibonacciActionGoal.h"
#include "actionlib_tutorials/FibonacciActionResult.h"
#include "actionlib_tutorials/FibonacciActionFeedback.h"

namespace actionlib_tutorials
{

  class FibonacciAction : public ros::Msg
  {
    public:
      typedef actionlib_tutorials::FibonacciActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef actionlib_tutorials::FibonacciActionResult _action_result_type;
      _action_result_type action_result;
      typedef actionlib_tutorials::FibonacciActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    FibonacciAction():
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

    virtual const char * getType() override { return "actionlib_tutorials/FibonacciAction"; };
    virtual const char * getMD5() override { return "f59df5767bf7634684781c92598b2406"; };

  };

}
#endif
