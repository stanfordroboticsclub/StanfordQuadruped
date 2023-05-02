#ifndef _ROS_SERVICE_NodeletList_h
#define _ROS_SERVICE_NodeletList_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace nodelet
{

static const char NODELETLIST[] = "nodelet/NodeletList";

  class NodeletListRequest : public ros::Msg
  {
    public:

    NodeletListRequest()
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

    virtual const char * getType() override { return NODELETLIST; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class NodeletListResponse : public ros::Msg
  {
    public:
      uint32_t nodelets_length;
      typedef char* _nodelets_type;
      _nodelets_type st_nodelets;
      _nodelets_type * nodelets;

    NodeletListResponse():
      nodelets_length(0), st_nodelets(), nodelets(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->nodelets_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nodelets_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->nodelets_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->nodelets_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nodelets_length);
      for( uint32_t i = 0; i < nodelets_length; i++){
      uint32_t length_nodeletsi = strlen(this->nodelets[i]);
      varToArr(outbuffer + offset, length_nodeletsi);
      offset += 4;
      memcpy(outbuffer + offset, this->nodelets[i], length_nodeletsi);
      offset += length_nodeletsi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t nodelets_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      nodelets_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      nodelets_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      nodelets_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->nodelets_length);
      if(nodelets_lengthT > nodelets_length)
        this->nodelets = (char**)realloc(this->nodelets, nodelets_lengthT * sizeof(char*));
      nodelets_length = nodelets_lengthT;
      for( uint32_t i = 0; i < nodelets_length; i++){
      uint32_t length_st_nodelets;
      arrToVar(length_st_nodelets, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_nodelets; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_nodelets-1]=0;
      this->st_nodelets = (char *)(inbuffer + offset-1);
      offset += length_st_nodelets;
        memcpy( &(this->nodelets[i]), &(this->st_nodelets), sizeof(char*));
      }
     return offset;
    }

    virtual const char * getType() override { return NODELETLIST; };
    virtual const char * getMD5() override { return "99c7b10e794f5600b8030e697e946ca7"; };

  };

  class NodeletList {
    public:
    typedef NodeletListRequest Request;
    typedef NodeletListResponse Response;
  };

}
#endif
