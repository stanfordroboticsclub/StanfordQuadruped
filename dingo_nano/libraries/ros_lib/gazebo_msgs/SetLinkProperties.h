#ifndef _ROS_SERVICE_SetLinkProperties_h
#define _ROS_SERVICE_SetLinkProperties_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace gazebo_msgs
{

static const char SETLINKPROPERTIES[] = "gazebo_msgs/SetLinkProperties";

  class SetLinkPropertiesRequest : public ros::Msg
  {
    public:
      typedef const char* _link_name_type;
      _link_name_type link_name;
      typedef geometry_msgs::Pose _com_type;
      _com_type com;
      typedef bool _gravity_mode_type;
      _gravity_mode_type gravity_mode;
      typedef float _mass_type;
      _mass_type mass;
      typedef float _ixx_type;
      _ixx_type ixx;
      typedef float _ixy_type;
      _ixy_type ixy;
      typedef float _ixz_type;
      _ixz_type ixz;
      typedef float _iyy_type;
      _iyy_type iyy;
      typedef float _iyz_type;
      _iyz_type iyz;
      typedef float _izz_type;
      _izz_type izz;

    SetLinkPropertiesRequest():
      link_name(""),
      com(),
      gravity_mode(0),
      mass(0),
      ixx(0),
      ixy(0),
      ixz(0),
      iyy(0),
      iyz(0),
      izz(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_link_name = strlen(this->link_name);
      varToArr(outbuffer + offset, length_link_name);
      offset += 4;
      memcpy(outbuffer + offset, this->link_name, length_link_name);
      offset += length_link_name;
      offset += this->com.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_gravity_mode;
      u_gravity_mode.real = this->gravity_mode;
      *(outbuffer + offset + 0) = (u_gravity_mode.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gravity_mode);
      offset += serializeAvrFloat64(outbuffer + offset, this->mass);
      offset += serializeAvrFloat64(outbuffer + offset, this->ixx);
      offset += serializeAvrFloat64(outbuffer + offset, this->ixy);
      offset += serializeAvrFloat64(outbuffer + offset, this->ixz);
      offset += serializeAvrFloat64(outbuffer + offset, this->iyy);
      offset += serializeAvrFloat64(outbuffer + offset, this->iyz);
      offset += serializeAvrFloat64(outbuffer + offset, this->izz);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_link_name;
      arrToVar(length_link_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_link_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_link_name-1]=0;
      this->link_name = (char *)(inbuffer + offset-1);
      offset += length_link_name;
      offset += this->com.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_gravity_mode;
      u_gravity_mode.base = 0;
      u_gravity_mode.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gravity_mode = u_gravity_mode.real;
      offset += sizeof(this->gravity_mode);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mass));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ixx));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ixy));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ixz));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->iyy));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->iyz));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->izz));
     return offset;
    }

    virtual const char * getType() override { return SETLINKPROPERTIES; };
    virtual const char * getMD5() override { return "68ac74a4be01b165bc305b5ccdc45e91"; };

  };

  class SetLinkPropertiesResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _status_message_type;
      _status_message_type status_message;

    SetLinkPropertiesResponse():
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

    virtual const char * getType() override { return SETLINKPROPERTIES; };
    virtual const char * getMD5() override { return "2ec6f3eff0161f4257b808b12bc830c2"; };

  };

  class SetLinkProperties {
    public:
    typedef SetLinkPropertiesRequest Request;
    typedef SetLinkPropertiesResponse Response;
  };

}
#endif
