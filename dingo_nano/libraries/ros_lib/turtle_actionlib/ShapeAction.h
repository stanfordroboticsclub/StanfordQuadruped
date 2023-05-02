#ifndef _ROS_turtle_actionlib_ShapeAction_h
#define _ROS_turtle_actionlib_ShapeAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "turtle_actionlib/ShapeActionGoal.h"
#include "turtle_actionlib/ShapeActionResult.h"
#include "turtle_actionlib/ShapeActionFeedback.h"

namespace turtle_actionlib
{

  class ShapeAction : public ros::Msg
  {
    public:
      typedef turtle_actionlib::ShapeActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef turtle_actionlib::ShapeActionResult _action_result_type;
      _action_result_type action_result;
      typedef turtle_actionlib::ShapeActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    ShapeAction():
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

    virtual const char * getType() override { return "turtle_actionlib/ShapeAction"; };
    virtual const char * getMD5() override { return "d73b17d6237a925511f5d7727a1dc903"; };

  };

}
#endif
