#ifndef _ROS_control_msgs_GripperCommand_h
#define _ROS_control_msgs_GripperCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"

namespace control_msgs
{

  class GripperCommand : public ros::Msg
  {
    public:
      typedef float _position_type;
      _position_type position;
      typedef float _max_effort_type;
      _max_effort_type max_effort;

    GripperCommand():
      position(0),
      max_effort(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->position);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_effort);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->position));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_effort));
     return offset;
    }

    #ifdef ESP8266
        const char * getType() { return  ("control_msgs/GripperCommand");};
    #else
        const char * getType() { return  PSTR("control_msgs/GripperCommand");};
    #endif
    #ifdef ESP8266
        const char * getMD5() { return  ("680acaff79486f017132a7f198d40f08");};
    #else
        const char * getMD5() { return  PSTR("680acaff79486f017132a7f198d40f08");};
    #endif

  };

}
#endif
