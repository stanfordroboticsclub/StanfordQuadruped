#ifndef _ROS_tf2_msgs_TF2Error_h
#define _ROS_tf2_msgs_TF2Error_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tf2_msgs
{

  class TF2Error : public ros::Msg
  {
    public:
      typedef uint8_t _error_type;
      _error_type error;
      typedef const char* _error_string_type;
      _error_string_type error_string;
      enum { NO_ERROR =  0 };
      enum { LOOKUP_ERROR =  1 };
      enum { CONNECTIVITY_ERROR =  2 };
      enum { EXTRAPOLATION_ERROR =  3 };
      enum { INVALID_ARGUMENT_ERROR =  4 };
      enum { TIMEOUT_ERROR =  5 };
      enum { TRANSFORM_ERROR =  6 };

    TF2Error():
      error(0),
      error_string("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->error >> (8 * 0)) & 0xFF;
      offset += sizeof(this->error);
      uint32_t length_error_string = strlen(this->error_string);
      varToArr(outbuffer + offset, length_error_string);
      offset += 4;
      memcpy(outbuffer + offset, this->error_string, length_error_string);
      offset += length_error_string;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->error =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->error);
      uint32_t length_error_string;
      arrToVar(length_error_string, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_error_string; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_error_string-1]=0;
      this->error_string = (char *)(inbuffer + offset-1);
      offset += length_error_string;
     return offset;
    }

    virtual const char * getType() override { return "tf2_msgs/TF2Error"; };
    virtual const char * getMD5() override { return "bc6848fd6fd750c92e38575618a4917d"; };

  };

}
#endif
