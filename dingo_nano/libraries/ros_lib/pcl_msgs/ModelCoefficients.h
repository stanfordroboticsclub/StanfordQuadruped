#ifndef _ROS_pcl_msgs_ModelCoefficients_h
#define _ROS_pcl_msgs_ModelCoefficients_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace pcl_msgs
{

  class ModelCoefficients : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t values_length;
      typedef float _values_type;
      _values_type st_values;
      _values_type * values;

    ModelCoefficients():
      header(),
      values_length(0), st_values(), values(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->values_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->values_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->values_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->values_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->values_length);
      for( uint32_t i = 0; i < values_length; i++){
      union {
        float real;
        uint32_t base;
      } u_valuesi;
      u_valuesi.real = this->values[i];
      *(outbuffer + offset + 0) = (u_valuesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_valuesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_valuesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_valuesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->values[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t values_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      values_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      values_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      values_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->values_length);
      if(values_lengthT > values_length)
        this->values = (float*)realloc(this->values, values_lengthT * sizeof(float));
      values_length = values_lengthT;
      for( uint32_t i = 0; i < values_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_values;
      u_st_values.base = 0;
      u_st_values.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_values.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_values.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_values.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_values = u_st_values.real;
      offset += sizeof(this->st_values);
        memcpy( &(this->values[i]), &(this->st_values), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "pcl_msgs/ModelCoefficients"; };
    virtual const char * getMD5() override { return "ca27dea75e72cb894cd36f9e5005e93e"; };

  };

}
#endif
