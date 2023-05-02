#ifndef _ROS_pcl_msgs_PointIndices_h
#define _ROS_pcl_msgs_PointIndices_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace pcl_msgs
{

  class PointIndices : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t indices_length;
      typedef int32_t _indices_type;
      _indices_type st_indices;
      _indices_type * indices;

    PointIndices():
      header(),
      indices_length(0), st_indices(), indices(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->indices_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->indices_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->indices_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->indices_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->indices_length);
      for( uint32_t i = 0; i < indices_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_indicesi;
      u_indicesi.real = this->indices[i];
      *(outbuffer + offset + 0) = (u_indicesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_indicesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_indicesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_indicesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->indices[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t indices_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      indices_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      indices_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      indices_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->indices_length);
      if(indices_lengthT > indices_length)
        this->indices = (int32_t*)realloc(this->indices, indices_lengthT * sizeof(int32_t));
      indices_length = indices_lengthT;
      for( uint32_t i = 0; i < indices_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_indices;
      u_st_indices.base = 0;
      u_st_indices.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_indices.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_indices.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_indices.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_indices = u_st_indices.real;
      offset += sizeof(this->st_indices);
        memcpy( &(this->indices[i]), &(this->st_indices), sizeof(int32_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "pcl_msgs/PointIndices"; };
    virtual const char * getMD5() override { return "458c7998b7eaf99908256472e273b3d4"; };

  };

}
#endif
