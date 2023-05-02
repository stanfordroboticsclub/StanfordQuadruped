#ifndef _ROS_std_msgs_UInt64_h
#define _ROS_std_msgs_UInt64_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace std_msgs
{

  class UInt64 : public ros::Msg
  {
    public:
      typedef uint64_t _data_type;
      _data_type data;

    UInt64():
      data(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->data >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->data >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->data >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->data >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->data >> (8 * 7)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->data =  ((uint64_t) (*(inbuffer + offset)));
      this->data |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->data |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->data |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->data |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->data |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->data |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->data);
     return offset;
    }

    virtual const char * getType() override { return "std_msgs/UInt64"; };
    virtual const char * getMD5() override { return "1b2a79973e8bf53d7b53acb71299cb57"; };

  };

}
#endif
