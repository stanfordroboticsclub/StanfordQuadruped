#ifndef _ROS_actionlib_tutorials_AveragingActionFeedback_h
#define _ROS_actionlib_tutorials_AveragingActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "actionlib_tutorials/AveragingFeedback.h"

namespace actionlib_tutorials
{

  class AveragingActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef actionlib_tutorials::AveragingFeedback _feedback_type;
      _feedback_type feedback;

    AveragingActionFeedback():
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
        const char * getType() { return  ("actionlib_tutorials/AveragingActionFeedback");};
    #else
        const char * getType() { return  PSTR("actionlib_tutorials/AveragingActionFeedback");};
    #endif
    #ifdef ESP8266
        const char * getMD5() { return  ("78a4a09241b1791069223ae7ebd5b16b");};
    #else
        const char * getMD5() { return  PSTR("78a4a09241b1791069223ae7ebd5b16b");};
    #endif

  };

}
#endif
