#ifndef _ROS_SERVICE_GetMapROI_h
#define _ROS_SERVICE_GetMapROI_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"
#include "nav_msgs/OccupancyGrid.h"

namespace map_msgs
{

#ifdef ESP8266
    static const char GETMAPROI[] = "map_msgs/GetMapROI";
#else
    static const char GETMAPROI[] PROGMEM = "map_msgs/GetMapROI";
#endif

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

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      offset += serializeAvrFloat64(outbuffer + offset, this->l_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->l_y);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->l_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->l_y));
     return offset;
    }

    const char * getType(){ return GETMAPROI; };
    #ifdef ESP8266
        const char * getMD5() { return  ("43c2ff8f45af555c0eaf070c401e9a47");};
    #else
        const char * getMD5() { return  PSTR("43c2ff8f45af555c0eaf070c401e9a47");};
    #endif

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

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->sub_map.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->sub_map.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETMAPROI; };
    #ifdef ESP8266
        const char * getMD5() { return  ("4d1986519c00d81967d2891a606b234c");};
    #else
        const char * getMD5() { return  PSTR("4d1986519c00d81967d2891a606b234c");};
    #endif

  };

  class GetMapROI {
    public:
    typedef GetMapROIRequest Request;
    typedef GetMapROIResponse Response;
  };

}
#endif
