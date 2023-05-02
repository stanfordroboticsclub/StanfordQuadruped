#ifndef _ROS_SERVICE_GetPointMapROI_h
#define _ROS_SERVICE_GetPointMapROI_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/PointCloud2.h"

namespace map_msgs
{

static const char GETPOINTMAPROI[] = "map_msgs/GetPointMapROI";

  class GetPointMapROIRequest : public ros::Msg
  {
    public:
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _z_type;
      _z_type z;
      typedef float _r_type;
      _r_type r;
      typedef float _l_x_type;
      _l_x_type l_x;
      typedef float _l_y_type;
      _l_y_type l_y;
      typedef float _l_z_type;
      _l_z_type l_z;

    GetPointMapROIRequest():
      x(0),
      y(0),
      z(0),
      r(0),
      l_x(0),
      l_y(0),
      l_z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      offset += serializeAvrFloat64(outbuffer + offset, this->z);
      offset += serializeAvrFloat64(outbuffer + offset, this->r);
      offset += serializeAvrFloat64(outbuffer + offset, this->l_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->l_y);
      offset += serializeAvrFloat64(outbuffer + offset, this->l_z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->z));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->r));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->l_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->l_y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->l_z));
     return offset;
    }

    virtual const char * getType() override { return GETPOINTMAPROI; };
    virtual const char * getMD5() override { return "895f7e437a9a6dd225316872b187a303"; };

  };

  class GetPointMapROIResponse : public ros::Msg
  {
    public:
      typedef sensor_msgs::PointCloud2 _sub_map_type;
      _sub_map_type sub_map;

    GetPointMapROIResponse():
      sub_map()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->sub_map.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->sub_map.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return GETPOINTMAPROI; };
    virtual const char * getMD5() override { return "313769f8b0e724525c6463336cbccd63"; };

  };

  class GetPointMapROI {
    public:
    typedef GetPointMapROIRequest Request;
    typedef GetPointMapROIResponse Response;
  };

}
#endif
