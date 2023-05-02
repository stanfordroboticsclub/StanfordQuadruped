#ifndef _ROS_rospy_tutorials_HeaderString_h
#define _ROS_rospy_tutorials_HeaderString_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"
#include "std_msgs/Header.h"

namespace rospy_tutorials
{

  class HeaderString : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _data_type;
      _data_type data;

    HeaderString():
      header(),
      data("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_data = strlen(this->data);
      varToArr(outbuffer + offset, length_data);
      offset += 4;
      memcpy(outbuffer + offset, this->data, length_data);
      offset += length_data;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_data;
      arrToVar(length_data, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_data; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_data-1]=0;
      this->data = (char *)(inbuffer + offset-1);
      offset += length_data;
     return offset;
    }

    #ifdef ESP8266
        const char * getType() { return  ("rospy_tutorials/HeaderString");};
    #else
        const char * getType() { return  PSTR("rospy_tutorials/HeaderString");};
    #endif
    #ifdef ESP8266
        const char * getMD5() { return  ("c99a9440709e4d4a9716d55b8270d5e7");};
    #else
        const char * getMD5() { return  PSTR("c99a9440709e4d4a9716d55b8270d5e7");};
    #endif

  };

}
#endif
