#ifndef _ROS_std_msgs_Int16_h
#define _ROS_std_msgs_Int16_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace std_msgs
{

  class Int16 : public ros::Msg
  {
    public:
      typedef int16_t _data_type;
      _data_type data;

    Int16():
      data(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_data;
      u_data.real = this->data;
      *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_data;
      u_data.base = 0;
      u_data.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_data.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->data = u_data.real;
      offset += sizeof(this->data);
     return offset;
    }

    virtual const char * getType() override { return "std_msgs/Int16"; };
    virtual const char * getMD5() override { return "8524586e34fbd7cb1c08c5f5f1ca0e57"; };

  };

}
#endif
