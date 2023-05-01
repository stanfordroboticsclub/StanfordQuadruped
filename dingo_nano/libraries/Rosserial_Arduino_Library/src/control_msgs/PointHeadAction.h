#ifndef _ROS_control_msgs_PointHeadAction_h
#define _ROS_control_msgs_PointHeadAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"
#include "control_msgs/PointHeadActionGoal.h"
#include "control_msgs/PointHeadActionResult.h"
#include "control_msgs/PointHeadActionFeedback.h"

namespace control_msgs
{

  class PointHeadAction : public ros::Msg
  {
    public:
      typedef control_msgs::PointHeadActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef control_msgs::PointHeadActionResult _action_result_type;
      _action_result_type action_result;
      typedef control_msgs::PointHeadActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    PointHeadAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    #ifdef ESP8266
        const char * getType() { return  ("control_msgs/PointHeadAction");};
    #else
        const char * getType() { return  PSTR("control_msgs/PointHeadAction");};
    #endif
    #ifdef ESP8266
        const char * getMD5() { return  ("7252920f1243de1b741f14f214125371");};
    #else
        const char * getMD5() { return  PSTR("7252920f1243de1b741f14f214125371");};
    #endif

  };

}
#endif
