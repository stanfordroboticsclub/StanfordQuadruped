#ifndef _ROS_turtlesim_Color_h
#define _ROS_turtlesim_Color_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"

namespace turtlesim
{

  class Color : public ros::Msg
  {
    public:
      typedef uint8_t _r_type;
      _r_type r;
      typedef uint8_t _g_type;
      _g_type g;
      typedef uint8_t _b_type;
      _b_type b;

    Color():
      r(0),
      g(0),
      b(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->r >> (8 * 0)) & 0xFF;
      offset += sizeof(this->r);
      *(outbuffer + offset + 0) = (this->g >> (8 * 0)) & 0xFF;
      offset += sizeof(this->g);
      *(outbuffer + offset + 0) = (this->b >> (8 * 0)) & 0xFF;
      offset += sizeof(this->b);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->r =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->r);
      this->g =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->g);
      this->b =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->b);
     return offset;
    }

    #ifdef ESP8266
        const char * getType() { return  ("turtlesim/Color");};
    #else
        const char * getType() { return  PSTR("turtlesim/Color");};
    #endif
    #ifdef ESP8266
        const char * getMD5() { return  ("353891e354491c51aabe32df673fb446");};
    #else
        const char * getMD5() { return  PSTR("353891e354491c51aabe32df673fb446");};
    #endif

  };

}
#endif
