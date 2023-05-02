#ifndef _ROS_SERVICE_FrameGraph_h
#define _ROS_SERVICE_FrameGraph_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tf2_msgs
{

static const char FRAMEGRAPH[] = "tf2_msgs/FrameGraph";

  class FrameGraphRequest : public ros::Msg
  {
    public:

    FrameGraphRequest()
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

    virtual const char * getType() override { return FRAMEGRAPH; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class FrameGraphResponse : public ros::Msg
  {
    public:
      typedef const char* _frame_yaml_type;
      _frame_yaml_type frame_yaml;

    FrameGraphResponse():
      frame_yaml("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_frame_yaml = strlen(this->frame_yaml);
      varToArr(outbuffer + offset, length_frame_yaml);
      offset += 4;
      memcpy(outbuffer + offset, this->frame_yaml, length_frame_yaml);
      offset += length_frame_yaml;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_frame_yaml;
      arrToVar(length_frame_yaml, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_frame_yaml; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_frame_yaml-1]=0;
      this->frame_yaml = (char *)(inbuffer + offset-1);
      offset += length_frame_yaml;
     return offset;
    }

    virtual const char * getType() override { return FRAMEGRAPH; };
    virtual const char * getMD5() override { return "437ea58e9463815a0d511c7326b686b0"; };

  };

  class FrameGraph {
    public:
    typedef FrameGraphRequest Request;
    typedef FrameGraphResponse Response;
  };

}
#endif
