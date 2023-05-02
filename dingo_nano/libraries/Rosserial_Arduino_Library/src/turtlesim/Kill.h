#ifndef _ROS_SERVICE_Kill_h
#define _ROS_SERVICE_Kill_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"

namespace turtlesim
{

#ifdef ESP8266
    static const char KILL[] = "turtlesim/Kill";
#else
    static const char KILL[] PROGMEM = "turtlesim/Kill";
#endif

  class KillRequest : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;

    KillRequest():
      name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
     return offset;
    }

    const char * getType(){ return KILL; };
    #ifdef ESP8266
        const char * getMD5() { return  ("c1f3d28f1b044c871e6eff2e9fc3c667");};
    #else
        const char * getMD5() { return  PSTR("c1f3d28f1b044c871e6eff2e9fc3c667");};
    #endif

  };

  class KillResponse : public ros::Msg
  {
    public:

    KillResponse()
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

    const char * getType(){ return KILL; };
    #ifdef ESP8266
        const char * getMD5() { return  ("d41d8cd98f00b204e9800998ecf8427e");};
    #else
        const char * getMD5() { return  PSTR("d41d8cd98f00b204e9800998ecf8427e");};
    #endif

  };

  class Kill {
    public:
    typedef KillRequest Request;
    typedef KillResponse Response;
  };

}
#endif
