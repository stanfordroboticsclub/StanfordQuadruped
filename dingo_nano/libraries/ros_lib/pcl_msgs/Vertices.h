#ifndef _ROS_pcl_msgs_Vertices_h
#define _ROS_pcl_msgs_Vertices_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pcl_msgs
{

  class Vertices : public ros::Msg
  {
    public:
      uint32_t vertices_length;
      typedef uint32_t _vertices_type;
      _vertices_type st_vertices;
      _vertices_type * vertices;

    Vertices():
      vertices_length(0), st_vertices(), vertices(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->vertices_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vertices_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->vertices_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->vertices_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vertices_length);
      for( uint32_t i = 0; i < vertices_length; i++){
      *(outbuffer + offset + 0) = (this->vertices[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vertices[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->vertices[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->vertices[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vertices[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t vertices_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      vertices_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      vertices_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      vertices_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->vertices_length);
      if(vertices_lengthT > vertices_length)
        this->vertices = (uint32_t*)realloc(this->vertices, vertices_lengthT * sizeof(uint32_t));
      vertices_length = vertices_lengthT;
      for( uint32_t i = 0; i < vertices_length; i++){
      this->st_vertices =  ((uint32_t) (*(inbuffer + offset)));
      this->st_vertices |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_vertices |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_vertices |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_vertices);
        memcpy( &(this->vertices[i]), &(this->st_vertices), sizeof(uint32_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "pcl_msgs/Vertices"; };
    virtual const char * getMD5() override { return "39bd7b1c23763ddd1b882b97cb7cfe11"; };

  };

}
#endif
