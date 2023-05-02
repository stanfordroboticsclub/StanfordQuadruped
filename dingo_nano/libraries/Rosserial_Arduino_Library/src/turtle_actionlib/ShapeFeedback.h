#ifndef _ROS_turtle_actionlib_ShapeFeedback_h
#define _ROS_turtle_actionlib_ShapeFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"

namespace turtle_actionlib
{

  class ShapeFeedback : public ros::Msg
  {
    public:

    ShapeFeedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    #ifdef ESP8266
        const char * getType() { return  ("turtle_actionlib/ShapeFeedback");};
    #else
        const char * getType() { return  PSTR("turtle_actionlib/ShapeFeedback");};
    #endif
    #ifdef ESP8266
        const char * getMD5() { return  ("d41d8cd98f00b204e9800998ecf8427e");};
    #else
        const char * getMD5() { return  PSTR("d41d8cd98f00b204e9800998ecf8427e");};
    #endif

  };

}
#endif
