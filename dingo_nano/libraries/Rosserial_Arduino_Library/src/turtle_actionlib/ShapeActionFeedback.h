#ifndef _ROS_turtle_actionlib_ShapeActionFeedback_h
#define _ROS_turtle_actionlib_ShapeActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "turtle_actionlib/ShapeFeedback.h"

namespace turtle_actionlib
{

  class ShapeActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef turtle_actionlib::ShapeFeedback _feedback_type;
      _feedback_type feedback;

    ShapeActionFeedback():
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
        const char * getType() { return  ("turtle_actionlib/ShapeActionFeedback");};
    #else
        const char * getType() { return  PSTR("turtle_actionlib/ShapeActionFeedback");};
    #endif
    #ifdef ESP8266
        const char * getMD5() { return  ("aae20e09065c3809e8a8e87c4c8953fd");};
    #else
        const char * getMD5() { return  PSTR("aae20e09065c3809e8a8e87c4c8953fd");};
    #endif

  };

}
#endif
