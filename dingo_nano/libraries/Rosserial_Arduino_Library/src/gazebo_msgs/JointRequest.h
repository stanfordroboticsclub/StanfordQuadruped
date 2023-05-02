#ifndef _ROS_SERVICE_JointRequest_h
#define _ROS_SERVICE_JointRequest_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"

namespace gazebo_msgs
{

#ifdef ESP8266
    static const char JOINTREQUEST[] = "gazebo_msgs/JointRequest";
#else
    static const char JOINTREQUEST[] PROGMEM = "gazebo_msgs/JointRequest";
#endif

  class JointRequestRequest : public ros::Msg
  {
    public:
      typedef const char* _joint_name_type;
      _joint_name_type joint_name;

    JointRequestRequest():
      joint_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_joint_name = strlen(this->joint_name);
      varToArr(outbuffer + offset, length_joint_name);
      offset += 4;
      memcpy(outbuffer + offset, this->joint_name, length_joint_name);
      offset += length_joint_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_joint_name;
      arrToVar(length_joint_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_joint_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_joint_name-1]=0;
      this->joint_name = (char *)(inbuffer + offset-1);
      offset += length_joint_name;
     return offset;
    }

    const char * getType(){ return JOINTREQUEST; };
    #ifdef ESP8266
        const char * getMD5() { return  ("0be1351618e1dc030eb7959d9a4902de");};
    #else
        const char * getMD5() { return  PSTR("0be1351618e1dc030eb7959d9a4902de");};
    #endif

  };

  class JointRequestResponse : public ros::Msg
  {
    public:

    JointRequestResponse()
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

    const char * getType(){ return JOINTREQUEST; };
    #ifdef ESP8266
        const char * getMD5() { return  ("d41d8cd98f00b204e9800998ecf8427e");};
    #else
        const char * getMD5() { return  PSTR("d41d8cd98f00b204e9800998ecf8427e");};
    #endif

  };

  class JointRequest {
    public:
    typedef JointRequestRequest Request;
    typedef JointRequestResponse Response;
  };

}
#endif
