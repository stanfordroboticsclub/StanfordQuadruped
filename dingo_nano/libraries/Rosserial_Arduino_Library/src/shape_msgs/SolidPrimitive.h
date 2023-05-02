#ifndef _ROS_shape_msgs_SolidPrimitive_h
#define _ROS_shape_msgs_SolidPrimitive_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace shape_msgs
{

  class SolidPrimitive : public ros::Msg
  {
    public:
      typedef uint8_t _type_type;
      _type_type type;
      uint32_t dimensions_length;
      typedef float _dimensions_type;
      _dimensions_type st_dimensions;
      _dimensions_type * dimensions;
      enum { BOX = 1 };
      enum { SPHERE = 2 };
      enum { CYLINDER = 3 };
      enum { CONE = 4 };
      enum { BOX_X = 0 };
      enum { BOX_Y = 1 };
      enum { BOX_Z = 2 };
      enum { SPHERE_RADIUS = 0 };
      enum { CYLINDER_HEIGHT = 0 };
      enum { CYLINDER_RADIUS = 1 };
      enum { CONE_HEIGHT = 0 };
      enum { CONE_RADIUS = 1 };

    SolidPrimitive():
      type(0),
      dimensions_length(0), st_dimensions(), dimensions(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      *(outbuffer + offset + 0) = (this->dimensions_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dimensions_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->dimensions_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->dimensions_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dimensions_length);
      for( uint32_t i = 0; i < dimensions_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->dimensions[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->type);
      uint32_t dimensions_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      dimensions_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      dimensions_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      dimensions_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->dimensions_length);
      if(dimensions_lengthT > dimensions_length)
        this->dimensions = (float*)realloc(this->dimensions, dimensions_lengthT * sizeof(float));
      dimensions_length = dimensions_lengthT;
      for( uint32_t i = 0; i < dimensions_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_dimensions));
        memcpy( &(this->dimensions[i]), &(this->st_dimensions), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "shape_msgs/SolidPrimitive"; };
    virtual const char * getMD5() override { return "d8f8cbc74c5ff283fca29569ccefb45d"; };

  };

}
#endif
