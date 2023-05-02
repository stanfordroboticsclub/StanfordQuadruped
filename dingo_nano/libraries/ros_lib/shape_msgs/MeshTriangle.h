#ifndef _ROS_shape_msgs_MeshTriangle_h
#define _ROS_shape_msgs_MeshTriangle_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace shape_msgs
{

  class MeshTriangle : public ros::Msg
  {
    public:
      uint32_t vertex_indices[3];

    MeshTriangle():
      vertex_indices()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 3; i++){
      *(outbuffer + offset + 0) = (this->vertex_indices[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vertex_indices[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->vertex_indices[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->vertex_indices[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vertex_indices[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 3; i++){
      this->vertex_indices[i] =  ((uint32_t) (*(inbuffer + offset)));
      this->vertex_indices[i] |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->vertex_indices[i] |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->vertex_indices[i] |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->vertex_indices[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "shape_msgs/MeshTriangle"; };
    virtual const char * getMD5() override { return "23688b2e6d2de3d32fe8af104a903253"; };

  };

}
#endif
