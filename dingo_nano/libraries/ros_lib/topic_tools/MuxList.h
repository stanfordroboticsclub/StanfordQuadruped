#ifndef _ROS_SERVICE_MuxList_h
#define _ROS_SERVICE_MuxList_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace topic_tools
{

static const char MUXLIST[] = "topic_tools/MuxList";

  class MuxListRequest : public ros::Msg
  {
    public:

    MuxListRequest()
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

    virtual const char * getType() override { return MUXLIST; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class MuxListResponse : public ros::Msg
  {
    public:
      uint32_t topics_length;
      typedef char* _topics_type;
      _topics_type st_topics;
      _topics_type * topics;

    MuxListResponse():
      topics_length(0), st_topics(), topics(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->topics_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->topics_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->topics_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->topics_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->topics_length);
      for( uint32_t i = 0; i < topics_length; i++){
      uint32_t length_topicsi = strlen(this->topics[i]);
      varToArr(outbuffer + offset, length_topicsi);
      offset += 4;
      memcpy(outbuffer + offset, this->topics[i], length_topicsi);
      offset += length_topicsi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t topics_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      topics_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      topics_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      topics_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->topics_length);
      if(topics_lengthT > topics_length)
        this->topics = (char**)realloc(this->topics, topics_lengthT * sizeof(char*));
      topics_length = topics_lengthT;
      for( uint32_t i = 0; i < topics_length; i++){
      uint32_t length_st_topics;
      arrToVar(length_st_topics, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_topics; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_topics-1]=0;
      this->st_topics = (char *)(inbuffer + offset-1);
      offset += length_st_topics;
        memcpy( &(this->topics[i]), &(this->st_topics), sizeof(char*));
      }
     return offset;
    }

    virtual const char * getType() override { return MUXLIST; };
    virtual const char * getMD5() override { return "b0eef9a05d4e829092fc2f2c3c2aad3d"; };

  };

  class MuxList {
    public:
    typedef MuxListRequest Request;
    typedef MuxListResponse Response;
  };

}
#endif
