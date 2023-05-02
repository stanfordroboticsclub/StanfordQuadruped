#ifndef _ROS_SERVICE_SetLinkState_h
#define _ROS_SERVICE_SetLinkState_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "gazebo_msgs/LinkState.h"

namespace gazebo_msgs
{

static const char SETLINKSTATE[] = "gazebo_msgs/SetLinkState";

  class SetLinkStateRequest : public ros::Msg
  {
    public:
      typedef gazebo_msgs::LinkState _link_state_type;
      _link_state_type link_state;

    SetLinkStateRequest():
      link_state()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->link_state.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->link_state.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SETLINKSTATE; };
    virtual const char * getMD5() override { return "22a2c757d56911b6f27868159e9a872d"; };

  };

  class SetLinkStateResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _status_message_type;
      _status_message_type status_message;

    SetLinkStateResponse():
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

    virtual const char * getType() override { return SETLINKSTATE; };
    virtual const char * getMD5() override { return "2ec6f3eff0161f4257b808b12bc830c2"; };

  };

  class SetLinkState {
    public:
    typedef SetLinkStateRequest Request;
    typedef SetLinkStateResponse Response;
  };

}
#endif
