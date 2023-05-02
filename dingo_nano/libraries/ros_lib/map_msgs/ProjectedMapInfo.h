#ifndef _ROS_map_msgs_ProjectedMapInfo_h
#define _ROS_map_msgs_ProjectedMapInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace map_msgs
{

  class ProjectedMapInfo : public ros::Msg
  {
    public:
      typedef const char* _frame_id_type;
      _frame_id_type frame_id;
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _width_type;
      _width_type width;
      typedef float _height_type;
      _height_type height;
      typedef float _min_z_type;
      _min_z_type min_z;
      typedef float _max_z_type;
      _max_z_type max_z;

    ProjectedMapInfo():
      frame_id(""),
      x(0),
      y(0),
      width(0),
      height(0),
      min_z(0),
      max_z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_frame_id = strlen(this->frame_id);
      varToArr(outbuffer + offset, length_frame_id);
      offset += 4;
      memcpy(outbuffer + offset, this->frame_id, length_frame_id);
      offset += length_frame_id;
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      offset += serializeAvrFloat64(outbuffer + offset, this->width);
      offset += serializeAvrFloat64(outbuffer + offset, this->height);
      offset += serializeAvrFloat64(outbuffer + offset, this->min_z);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_frame_id;
      arrToVar(length_frame_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_frame_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_frame_id-1]=0;
      this->frame_id = (char *)(inbuffer + offset-1);
      offset += length_frame_id;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->width));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->height));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->min_z));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_z));
     return offset;
    }

    virtual const char * getType() override { return "map_msgs/ProjectedMapInfo"; };
    virtual const char * getMD5() override { return "2dc10595ae94de23f22f8a6d2a0eef7a"; };

  };

}
#endif
