#ifndef _ROS_SERVICE_SetJointProperties_h
#define _ROS_SERVICE_SetJointProperties_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "gazebo_msgs/ODEJointProperties.h"

namespace gazebo_msgs
{

static const char SETJOINTPROPERTIES[] = "gazebo_msgs/SetJointProperties";

  class SetJointPropertiesRequest : public ros::Msg
  {
    public:
      typedef const char* _joint_name_type;
      _joint_name_type joint_name;
      typedef gazebo_msgs::ODEJointProperties _ode_joint_config_type;
      _ode_joint_config_type ode_joint_config;

    SetJointPropertiesRequest():
      joint_name(""),
      ode_joint_config()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_joint_name = strlen(this->joint_name);
      varToArr(outbuffer + offset, length_joint_name);
      offset += 4;
      memcpy(outbuffer + offset, this->joint_name, length_joint_name);
      offset += length_joint_name;
      offset += this->ode_joint_config.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_joint_name;
      arrToVar(length_joint_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_joint_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_joint_name-1]=0;
      this->joint_name = (char *)(inbuffer + offset-1);
      offset += length_joint_name;
      offset += this->ode_joint_config.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SETJOINTPROPERTIES; };
    virtual const char * getMD5() override { return "331fd8f35fd27e3c1421175590258e26"; };

  };

  class SetJointPropertiesResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _status_message_type;
      _status_message_type status_message;

    SetJointPropertiesResponse():
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

    virtual const char * getType() override { return SETJOINTPROPERTIES; };
    virtual const char * getMD5() override { return "2ec6f3eff0161f4257b808b12bc830c2"; };

  };

  class SetJointProperties {
    public:
    typedef SetJointPropertiesRequest Request;
    typedef SetJointPropertiesResponse Response;
  };

}
#endif
