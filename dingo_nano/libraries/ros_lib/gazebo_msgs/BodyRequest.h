#ifndef _ROS_SERVICE_BodyRequest_h
#define _ROS_SERVICE_BodyRequest_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gazebo_msgs
{

static const char BODYREQUEST[] = "gazebo_msgs/BodyRequest";

  class BodyRequestRequest : public ros::Msg
  {
    public:
      typedef const char* _body_name_type;
      _body_name_type body_name;

    BodyRequestRequest():
      body_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_body_name = strlen(this->body_name);
      varToArr(outbuffer + offset, length_body_name);
      offset += 4;
      memcpy(outbuffer + offset, this->body_name, length_body_name);
      offset += length_body_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_body_name;
      arrToVar(length_body_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_body_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_body_name-1]=0;
      this->body_name = (char *)(inbuffer + offset-1);
      offset += length_body_name;
     return offset;
    }

    virtual const char * getType() override { return BODYREQUEST; };
    virtual const char * getMD5() override { return "5eade9afe7f232d78005bd0cafeab755"; };

  };

  class BodyRequestResponse : public ros::Msg
  {
    public:

    BodyRequestResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return BODYREQUEST; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class BodyRequest {
    public:
    typedef BodyRequestRequest Request;
    typedef BodyRequestResponse Response;
  };

}
#endif
