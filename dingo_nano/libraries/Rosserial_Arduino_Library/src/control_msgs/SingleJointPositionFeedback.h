#ifndef _ROS_control_msgs_SingleJointPositionFeedback_h
#define _ROS_control_msgs_SingleJointPositionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"
#include "std_msgs/Header.h"

namespace control_msgs
{

  class SingleJointPositionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _position_type;
      _position_type position;
      typedef float _velocity_type;
      _velocity_type velocity;
      typedef float _error_type;
      _error_type error;

    SingleJointPositionFeedback():
      header(),
      position(0),
      velocity(0),
      error(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->position);
      offset += serializeAvrFloat64(outbuffer + offset, this->velocity);
      offset += serializeAvrFloat64(outbuffer + offset, this->error);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->position));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->velocity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->error));
     return offset;
    }

    #ifdef ESP8266
        const char * getType() { return  ("control_msgs/SingleJointPositionFeedback");};
    #else
        const char * getType() { return  PSTR("control_msgs/SingleJointPositionFeedback");};
    #endif
    #ifdef ESP8266
        const char * getMD5() { return  ("8cee65610a3d08e0a1bded82f146f1fd");};
    #else
        const char * getMD5() { return  PSTR("8cee65610a3d08e0a1bded82f146f1fd");};
    #endif

  };

}
#endif
