#ifndef _ROS_SERVICE_SetPhysicsProperties_h
#define _ROS_SERVICE_SetPhysicsProperties_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"
#include "gazebo_msgs/ODEPhysics.h"

namespace gazebo_msgs
{

static const char SETPHYSICSPROPERTIES[] = "gazebo_msgs/SetPhysicsProperties";

  class SetPhysicsPropertiesRequest : public ros::Msg
  {
    public:
      typedef float _time_step_type;
      _time_step_type time_step;
      typedef float _max_update_rate_type;
      _max_update_rate_type max_update_rate;
      typedef geometry_msgs::Vector3 _gravity_type;
      _gravity_type gravity;
      typedef gazebo_msgs::ODEPhysics _ode_config_type;
      _ode_config_type ode_config;

    SetPhysicsPropertiesRequest():
      time_step(0),
      max_update_rate(0),
      gravity(),
      ode_config()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->time_step);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_update_rate);
      offset += this->gravity.serialize(outbuffer + offset);
      offset += this->ode_config.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->time_step));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_update_rate));
      offset += this->gravity.deserialize(inbuffer + offset);
      offset += this->ode_config.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SETPHYSICSPROPERTIES; };
    virtual const char * getMD5() override { return "abd9f82732b52b92e9d6bb36e6a82452"; };

  };

  class SetPhysicsPropertiesResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _status_message_type;
      _status_message_type status_message;

    SetPhysicsPropertiesResponse():
      success(0),
      status_message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_status_message = strlen(this->status_message);
      varToArr(outbuffer + offset, length_status_message);
      offset += 4;
      memcpy(outbuffer + offset, this->status_message, length_status_message);
      offset += length_status_message;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_status_message;
      arrToVar(length_status_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_status_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_status_message-1]=0;
      this->status_message = (char *)(inbuffer + offset-1);
      offset += length_status_message;
     return offset;
    }

    virtual const char * getType() override { return SETPHYSICSPROPERTIES; };
    virtual const char * getMD5() override { return "2ec6f3eff0161f4257b808b12bc830c2"; };

  };

  class SetPhysicsProperties {
    public:
    typedef SetPhysicsPropertiesRequest Request;
    typedef SetPhysicsPropertiesResponse Response;
  };

}
#endif
