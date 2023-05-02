#ifndef _ROS_SERVICE_GetLinkState_h
#define _ROS_SERVICE_GetLinkState_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "gazebo_msgs/LinkState.h"

namespace gazebo_msgs
{

static const char GETLINKSTATE[] = "gazebo_msgs/GetLinkState";

  class GetLinkStateRequest : public ros::Msg
  {
    public:
      typedef const char* _link_name_type;
      _link_name_type link_name;
      typedef const char* _reference_frame_type;
      _reference_frame_type reference_frame;

    GetLinkStateRequest():
      link_name(""),
      reference_frame("")
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
      uint32_t length_reference_frame = strlen(this->reference_frame);
      varToArr(outbuffer + offset, length_reference_frame);
      offset += 4;
      memcpy(outbuffer + offset, this->reference_frame, length_reference_frame);
      offset += length_reference_frame;
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
      uint32_t length_reference_frame;
      arrToVar(length_reference_frame, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_reference_frame; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_reference_frame-1]=0;
      this->reference_frame = (char *)(inbuffer + offset-1);
      offset += length_reference_frame;
     return offset;
    }

    virtual const char * getType() override { return GETLINKSTATE; };
    virtual const char * getMD5() override { return "7551675c30aaa71f7c288d4864552001"; };

  };

  class GetLinkStateResponse : public ros::Msg
  {
    public:
      typedef gazebo_msgs::LinkState _link_state_type;
      _link_state_type link_state;
      typedef bool _success_type;
      _success_type success;
      typedef const char* _status_message_type;
      _status_message_type status_message;

    GetLinkStateResponse():
      link_state(),
      success(0),
      status_message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->link_state.serialize(outbuffer + offset);
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
      offset += this->link_state.deserialize(inbuffer + offset);
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

    virtual const char * getType() override { return GETLINKSTATE; };
    virtual const char * getMD5() override { return "8ba55ad34f9c072e75c0de57b089753b"; };

  };

  class GetLinkState {
    public:
    typedef GetLinkStateRequest Request;
    typedef GetLinkStateResponse Response;
  };

}
#endif
