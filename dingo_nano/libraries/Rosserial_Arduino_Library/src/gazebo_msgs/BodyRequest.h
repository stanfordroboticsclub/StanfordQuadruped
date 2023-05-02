#ifndef _ROS_SERVICE_BodyRequest_h
#define _ROS_SERVICE_BodyRequest_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"

namespace gazebo_msgs
{

#ifdef ESP8266
    static const char BODYREQUEST[] = "gazebo_msgs/BodyRequest";
#else
    static const char BODYREQUEST[] PROGMEM = "gazebo_msgs/BodyRequest";
#endif

  class BodyRequestRequest : public ros::Msg
  {
    public:
      typedef const char* _body_name_type;
      _body_name_type body_name;

    BodyRequestRequest():
      body_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_body_name = strlen(this->body_name);
      varToArr(outbuffer + offset, length_body_name);
      offset += 4;
      memcpy(outbuffer + offset, this->body_name, length_body_name);
      offset += length_body_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_body_name;
      arrToVar(length_body_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_body_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_body_name-1]=0;
      this->body_name = (char *)(inbuffer + offset-1);
      offset += length_body_name;
     return offset;
    }

    const char * getType(){ return BODYREQUEST; };
    #ifdef ESP8266
        const char * getMD5() { return  ("5eade9afe7f232d78005bd0cafeab755");};
    #else
        const char * getMD5() { return  PSTR("5eade9afe7f232d78005bd0cafeab755");};
    #endif

  };

  class BodyRequestResponse : public ros::Msg
  {
    public:

    BodyRequestResponse()
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

    const char * getType(){ return BODYREQUEST; };
    #ifdef ESP8266
        const char * getMD5() { return  ("d41d8cd98f00b204e9800998ecf8427e");};
    #else
        const char * getMD5() { return  PSTR("d41d8cd98f00b204e9800998ecf8427e");};
    #endif

  };

  class BodyRequest {
    public:
    typedef BodyRequestRequest Request;
    typedef BodyRequestResponse Response;
  };

}
#endif
