#ifndef _ROS_SERVICE_ReloadControllerLibraries_h
#define _ROS_SERVICE_ReloadControllerLibraries_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace controller_manager_msgs
{

static const char RELOADCONTROLLERLIBRARIES[] = "controller_manager_msgs/ReloadControllerLibraries";

  class ReloadControllerLibrariesRequest : public ros::Msg
  {
    public:
      typedef bool _force_kill_type;
      _force_kill_type force_kill;

    ReloadControllerLibrariesRequest():
      force_kill(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
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

    virtual int deserialize(unsigned char *inbuffer) override
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

    virtual const char * getType() override { return RELOADCONTROLLERLIBRARIES; };
    virtual const char * getMD5() override { return "18442b59be9479097f11c543bddbac62"; };

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

    virtual int serialize(unsigned char *outbuffer) const override
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

    virtual int deserialize(unsigned char *inbuffer) override
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

    virtual const char * getType() override { return RELOADCONTROLLERLIBRARIES; };
    virtual const char * getMD5() override { return "6f6da3883749771fac40d6deb24a8c02"; };

  };

  class ReloadControllerLibraries {
    public:
    typedef ReloadControllerLibrariesRequest Request;
    typedef ReloadControllerLibrariesResponse Response;
  };

}
#endif
