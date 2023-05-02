#ifndef _ROS_control_msgs_PointHeadFeedback_h
#define _ROS_control_msgs_PointHeadFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"

namespace control_msgs
{

  class PointHeadFeedback : public ros::Msg
  {
    public:
      typedef float _pointing_angle_error_type;
      _pointing_angle_error_type pointing_angle_error;

    PointHeadFeedback():
      pointing_angle_error(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->pointing_angle_error);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pointing_angle_error));
     return offset;
    }

    #ifdef ESP8266
        const char * getType() { return  ("control_msgs/PointHeadFeedback");};
    #else
        const char * getType() { return  PSTR("control_msgs/PointHeadFeedback");};
    #endif
    #ifdef ESP8266
        const char * getMD5() { return  ("cce80d27fd763682da8805a73316cab4");};
    #else
        const char * getMD5() { return  PSTR("cce80d27fd763682da8805a73316cab4");};
    #endif

  };

}
#endif
