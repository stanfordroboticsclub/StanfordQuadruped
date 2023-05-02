#ifndef _ROS_SERVICE_SendFilePath_h
#define _ROS_SERVICE_SendFilePath_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/String.h"

namespace rviz
{

static const char SENDFILEPATH[] = "rviz/SendFilePath";

  class SendFilePathRequest : public ros::Msg
  {
    public:
      typedef std_msgs::String _path_type;
      _path_type path;

    SendFilePathRequest():
      path()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->path.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->path.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SENDFILEPATH; };
    virtual const char * getMD5() override { return "8a631822f6e3078667af5e13f8ab06b7"; };

  };

  class SendFilePathResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SendFilePathResponse():
      success(0)
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
     return offset;
    }

    virtual const char * getType() override { return SENDFILEPATH; };
    virtual const char * getMD5() override { return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SendFilePath {
    public:
    typedef SendFilePathRequest Request;
    typedef SendFilePathResponse Response;
  };

}
#endif
