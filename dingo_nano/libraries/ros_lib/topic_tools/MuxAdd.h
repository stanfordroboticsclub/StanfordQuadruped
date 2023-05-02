#ifndef _ROS_SERVICE_MuxAdd_h
#define _ROS_SERVICE_MuxAdd_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace topic_tools
{

static const char MUXADD[] = "topic_tools/MuxAdd";

  class MuxAddRequest : public ros::Msg
  {
    public:
      typedef const char* _topic_type;
      _topic_type topic;

    MuxAddRequest():
      topic("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_topic = strlen(this->topic);
      varToArr(outbuffer + offset, length_topic);
      offset += 4;
      memcpy(outbuffer + offset, this->topic, length_topic);
      offset += length_topic;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_topic;
      arrToVar(length_topic, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_topic; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_topic-1]=0;
      this->topic = (char *)(inbuffer + offset-1);
      offset += length_topic;
     return offset;
    }

    virtual const char * getType() override { return MUXADD; };
    virtual const char * getMD5() override { return "d8f94bae31b356b24d0427f80426d0c3"; };

  };

  class MuxAddResponse : public ros::Msg
  {
    public:

    MuxAddResponse()
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

    virtual const char * getType() override { return MUXADD; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class MuxAdd {
    public:
    typedef MuxAddRequest Request;
    typedef MuxAddResponse Response;
  };

}
#endif
