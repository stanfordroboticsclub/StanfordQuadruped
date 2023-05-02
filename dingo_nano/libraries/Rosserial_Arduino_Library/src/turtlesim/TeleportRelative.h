#ifndef _ROS_SERVICE_TeleportRelative_h
#define _ROS_SERVICE_TeleportRelative_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"

namespace turtlesim
{

#ifdef ESP8266
    static const char TELEPORTRELATIVE[] = "turtlesim/TeleportRelative";
#else
    static const char TELEPORTRELATIVE[] PROGMEM = "turtlesim/TeleportRelative";
#endif

  class TeleportRelativeRequest : public ros::Msg
  {
    public:
      typedef float _linear_type;
      _linear_type linear;
      typedef float _angular_type;
      _angular_type angular;

    TeleportRelativeRequest():
      linear(0),
      angular(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_linear;
      u_linear.real = this->linear;
      *(outbuffer + offset + 0) = (u_linear.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_linear.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_linear.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_linear.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->linear);
      union {
        float real;
        uint32_t base;
      } u_angular;
      u_angular.real = this->angular;
      *(outbuffer + offset + 0) = (u_angular.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angular.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angular.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angular.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angular);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_linear;
      u_linear.base = 0;
      u_linear.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_linear.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_linear.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_linear.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->linear = u_linear.real;
      offset += sizeof(this->linear);
      union {
        float real;
        uint32_t base;
      } u_angular;
      u_angular.base = 0;
      u_angular.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angular.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angular.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angular.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angular = u_angular.real;
      offset += sizeof(this->angular);
     return offset;
    }

    const char * getType(){ return TELEPORTRELATIVE; };
    #ifdef ESP8266
        const char * getMD5() { return  ("9d5c2dcd348ac8f76ce2a4307bd63a13");};
    #else
        const char * getMD5() { return  PSTR("9d5c2dcd348ac8f76ce2a4307bd63a13");};
    #endif

  };

  class TeleportRelativeResponse : public ros::Msg
  {
    public:

    TeleportRelativeResponse()
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

    const char * getType(){ return TELEPORTRELATIVE; };
    #ifdef ESP8266
        const char * getMD5() { return  ("d41d8cd98f00b204e9800998ecf8427e");};
    #else
        const char * getMD5() { return  PSTR("d41d8cd98f00b204e9800998ecf8427e");};
    #endif

  };

  class TeleportRelative {
    public:
    typedef TeleportRelativeRequest Request;
    typedef TeleportRelativeResponse Response;
  };

}
#endif
