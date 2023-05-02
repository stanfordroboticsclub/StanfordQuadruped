#ifndef _ROS_actionlib_TwoIntsResult_h
#define _ROS_actionlib_TwoIntsResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace actionlib
{

  class TwoIntsResult : public ros::Msg
  {
    public:
      typedef int64_t _sum_type;
      _sum_type sum;

    TwoIntsResult():
      sum(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_sum;
      u_sum.real = this->sum;
      *(outbuffer + offset + 0) = (u_sum.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sum.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sum.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sum.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_sum.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_sum.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_sum.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_sum.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->sum);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_sum;
      u_sum.base = 0;
      u_sum.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sum.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sum.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sum.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_sum.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_sum.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_sum.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_sum.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->sum = u_sum.real;
      offset += sizeof(this->sum);
     return offset;
    }

    virtual const char * getType() override { return "actionlib/TwoIntsResult"; };
    virtual const char * getMD5() override { return "b88405221c77b1878a3cbbfff53428d7"; };

  };

}
#endif
