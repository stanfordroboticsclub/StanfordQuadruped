#ifndef _ROS_SERVICE_SetPen_h
#define _ROS_SERVICE_SetPen_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace turtlesim
{

static const char SETPEN[] = "turtlesim/SetPen";

  class SetPenRequest : public ros::Msg
  {
    public:
      typedef uint8_t _r_type;
      _r_type r;
      typedef uint8_t _g_type;
      _g_type g;
      typedef uint8_t _b_type;
      _b_type b;
      typedef uint8_t _width_type;
      _width_type width;
      typedef uint8_t _off_type;
      _off_type off;

    SetPenRequest():
      r(0),
      g(0),
      b(0),
      width(0),
      off(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->r >> (8 * 0)) & 0xFF;
      offset += sizeof(this->r);
      *(outbuffer + offset + 0) = (this->g >> (8 * 0)) & 0xFF;
      offset += sizeof(this->g);
      *(outbuffer + offset + 0) = (this->b >> (8 * 0)) & 0xFF;
      offset += sizeof(this->b);
      *(outbuffer + offset + 0) = (this->width >> (8 * 0)) & 0xFF;
      offset += sizeof(this->width);
      *(outbuffer + offset + 0) = (this->off >> (8 * 0)) & 0xFF;
      offset += sizeof(this->off);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->r =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->r);
      this->g =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->g);
      this->b =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->b);
      this->width =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->width);
      this->off =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->off);
     return offset;
    }

    virtual const char * getType() override { return SETPEN; };
    virtual const char * getMD5() override { return "9f452acce566bf0c0954594f69a8e41b"; };

  };

  class SetPenResponse : public ros::Msg
  {
    public:

    SetPenResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return SETPEN; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetPen {
    public:
    typedef SetPenRequest Request;
    typedef SetPenResponse Response;
  };

}
#endif
