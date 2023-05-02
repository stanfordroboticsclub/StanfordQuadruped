#ifndef _ROS_control_msgs_JointTolerance_h
#define _ROS_control_msgs_JointTolerance_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"

namespace control_msgs
{

  class JointTolerance : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef float _position_type;
      _position_type position;
      typedef float _velocity_type;
      _velocity_type velocity;
      typedef float _acceleration_type;
      _acceleration_type acceleration;

    JointTolerance():
      name(""),
      position(0),
      velocity(0),
      acceleration(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      offset += serializeAvrFloat64(outbuffer + offset, this->position);
      offset += serializeAvrFloat64(outbuffer + offset, this->velocity);
      offset += serializeAvrFloat64(outbuffer + offset, this->acceleration);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->position));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->velocity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->acceleration));
     return offset;
    }

    #ifdef ESP8266
        const char * getType() { return  ("control_msgs/JointTolerance");};
    #else
        const char * getType() { return  PSTR("control_msgs/JointTolerance");};
    #endif
    #ifdef ESP8266
        const char * getMD5() { return  ("f544fe9c16cf04547e135dd6063ff5be");};
    #else
        const char * getMD5() { return  PSTR("f544fe9c16cf04547e135dd6063ff5be");};
    #endif

  };

}
#endif
