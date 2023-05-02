#ifndef _ROS_control_msgs_SingleJointPositionActionFeedback_h
#define _ROS_control_msgs_SingleJointPositionActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "control_msgs/SingleJointPositionFeedback.h"

namespace control_msgs
{

  class SingleJointPositionActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef control_msgs::SingleJointPositionFeedback _feedback_type;
      _feedback_type feedback;

    SingleJointPositionActionFeedback():
      header(),
      status(),
      feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->feedback.deserialize(inbuffer + offset);
     return offset;
    }

    #ifdef ESP8266
        const char * getType() { return  ("control_msgs/SingleJointPositionActionFeedback");};
    #else
        const char * getType() { return  PSTR("control_msgs/SingleJointPositionActionFeedback");};
    #endif
    #ifdef ESP8266
        const char * getMD5() { return  ("3503b7cf8972f90d245850a5d8796cfa");};
    #else
        const char * getMD5() { return  PSTR("3503b7cf8972f90d245850a5d8796cfa");};
    #endif

  };

}
#endif
