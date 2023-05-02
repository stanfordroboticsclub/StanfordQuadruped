#ifndef _ROS_SERVICE_GetMapROI_h
#define _ROS_SERVICE_GetMapROI_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nav_msgs/OccupancyGrid.h"

namespace map_msgs
{

static const char GETMAPROI[] = "map_msgs/GetMapROI";

  class GetMapROIRequest : public ros::Msg
  {
    public:
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _l_x_type;
      _l_x_type l_x;
      typedef float _l_y_type;
      _l_y_type l_y;

    GetMapROIRequest():
      x(0),
      y(0),
      l_x(0),
      l_y(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      offset += serializeAvrFloat64(outbuffer + offset, this->l_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->l_y);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->l_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->l_y));
     return offset;
    }

    virtual const char * getType() override { return GETMAPROI; };
    virtual const char * getMD5() override { return "43c2ff8f45af555c0eaf070c401e9a47"; };

  };

  class GetMapROIResponse : public ros::Msg
  {
    public:
      typedef nav_msgs::OccupancyGrid _sub_map_type;
      _sub_map_type sub_map;

    GetMapROIResponse():
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

    virtual const char * getType() override { return GETMAPROI; };
    virtual const char * getMD5() override { return "4d1986519c00d81967d2891a606b234c"; };

  };

  class GetMapROI {
    public:
    typedef GetMapROIRequest Request;
    typedef GetMapROIResponse Response;
  };

}
#endif
