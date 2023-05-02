#ifndef _ROS_diagnostic_msgs_KeyValue_h
#define _ROS_diagnostic_msgs_KeyValue_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace diagnostic_msgs
{

  class KeyValue : public ros::Msg
  {
    public:
      typedef const char* _key_type;
      _key_type key;
      typedef const char* _value_type;
      _value_type value;

    KeyValue():
      key(""),
      value("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_key = strlen(this->key);
      varToArr(outbuffer + offset, length_key);
      offset += 4;
      memcpy(outbuffer + offset, this->key, length_key);
      offset += length_key;
      uint32_t length_value = strlen(this->value);
      varToArr(outbuffer + offset, length_value);
      offset += 4;
      memcpy(outbuffer + offset, this->value, length_value);
      offset += length_value;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_key;
      arrToVar(length_key, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_key; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_key-1]=0;
      this->key = (char *)(inbuffer + offset-1);
      offset += length_key;
      uint32_t length_value;
      arrToVar(length_value, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_value; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_value-1]=0;
      this->value = (char *)(inbuffer + offset-1);
      offset += length_value;
     return offset;
    }

    virtual const char * getType() override { return "diagnostic_msgs/KeyValue"; };
    virtual const char * getMD5() override { return "cf57fdc6617a881a88c16e768132149c"; };

  };

}
#endif
