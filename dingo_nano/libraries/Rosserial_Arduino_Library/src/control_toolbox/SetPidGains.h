#ifndef _ROS_SERVICE_SetPidGains_h
#define _ROS_SERVICE_SetPidGains_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"

namespace control_toolbox
{

#ifdef ESP8266
    static const char SETPIDGAINS[] = "control_toolbox/SetPidGains";
#else
    static const char SETPIDGAINS[] PROGMEM = "control_toolbox/SetPidGains";
#endif

  class SetPidGainsRequest : public ros::Msg
  {
    public:
      typedef float _p_type;
      _p_type p;
      typedef float _i_type;
      _i_type i;
      typedef float _d_type;
      _d_type d;
      typedef float _i_clamp_type;
      _i_clamp_type i_clamp;
      typedef bool _antiwindup_type;
      _antiwindup_type antiwindup;

    SetPidGainsRequest():
      p(0),
      i(0),
      d(0),
      i_clamp(0),
      antiwindup(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->p);
      offset += serializeAvrFloat64(outbuffer + offset, this->i);
      offset += serializeAvrFloat64(outbuffer + offset, this->d);
      offset += serializeAvrFloat64(outbuffer + offset, this->i_clamp);
      union {
        bool real;
        uint8_t base;
      } u_antiwindup;
      u_antiwindup.real = this->antiwindup;
      *(outbuffer + offset + 0) = (u_antiwindup.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->antiwindup);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->p));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->i));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->d));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->i_clamp));
      union {
        bool real;
        uint8_t base;
      } u_antiwindup;
      u_antiwindup.base = 0;
      u_antiwindup.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->antiwindup = u_antiwindup.real;
      offset += sizeof(this->antiwindup);
     return offset;
    }

    const char * getType(){ return SETPIDGAINS; };
    #ifdef ESP8266
        const char * getMD5() { return  ("4a43159879643e60937bf2893b633607");};
    #else
        const char * getMD5() { return  PSTR("4a43159879643e60937bf2893b633607");};
    #endif

  };

  class SetPidGainsResponse : public ros::Msg
  {
    public:

    SetPidGainsResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return SETPIDGAINS; };
    #ifdef ESP8266
        const char * getMD5() { return  ("d41d8cd98f00b204e9800998ecf8427e");};
    #else
        const char * getMD5() { return  PSTR("d41d8cd98f00b204e9800998ecf8427e");};
    #endif

  };

  class SetPidGains {
    public:
    typedef SetPidGainsRequest Request;
    typedef SetPidGainsResponse Response;
  };

}
#endif
