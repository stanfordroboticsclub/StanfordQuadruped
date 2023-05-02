#ifndef _ROS_SERVICE_ReloadControllerLibraries_h
#define _ROS_SERVICE_ReloadControllerLibraries_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"

namespace controller_manager_msgs
{

#ifdef ESP8266
    static const char RELOADCONTROLLERLIBRARIES[] = "controller_manager_msgs/ReloadControllerLibraries";
#else
    static const char RELOADCONTROLLERLIBRARIES[] PROGMEM = "controller_manager_msgs/ReloadControllerLibraries";
#endif

  class ReloadControllerLibrariesRequest : public ros::Msg
  {
    public:
      typedef bool _force_kill_type;
      _force_kill_type force_kill;

    ReloadControllerLibrariesRequest():
      force_kill(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_force_kill;
      u_force_kill.real = this->force_kill;
      *(outbuffer + offset + 0) = (u_force_kill.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->force_kill);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_force_kill;
      u_force_kill.base = 0;
      u_force_kill.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->force_kill = u_force_kill.real;
      offset += sizeof(this->force_kill);
     return offset;
    }

    const char * getType(){ return RELOADCONTROLLERLIBRARIES; };
    #ifdef ESP8266
        const char * getMD5() { return  ("18442b59be9479097f11c543bddbac62");};
    #else
        const char * getMD5() { return  PSTR("18442b59be9479097f11c543bddbac62");};
    #endif

  };

  class ReloadControllerLibrariesResponse : public ros::Msg
  {
    public:
      typedef bool _ok_type;
      _ok_type ok;

    ReloadControllerLibrariesResponse():
      ok(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ok;
      u_ok.real = this->ok;
      *(outbuffer + offset + 0) = (u_ok.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ok);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ok;
      u_ok.base = 0;
      u_ok.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ok = u_ok.real;
      offset += sizeof(this->ok);
     return offset;
    }

    const char * getType(){ return RELOADCONTROLLERLIBRARIES; };
    #ifdef ESP8266
        const char * getMD5() { return  ("6f6da3883749771fac40d6deb24a8c02");};
    #else
        const char * getMD5() { return  PSTR("6f6da3883749771fac40d6deb24a8c02");};
    #endif

  };

  class ReloadControllerLibraries {
    public:
    typedef ReloadControllerLibrariesRequest Request;
    typedef ReloadControllerLibrariesResponse Response;
  };

}
#endif
