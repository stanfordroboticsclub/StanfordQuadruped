#ifndef _ROS_actionlib_tutorials_FibonacciGoal_h
#define _ROS_actionlib_tutorials_FibonacciGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace actionlib_tutorials
{

  class FibonacciGoal : public ros::Msg
  {
    public:
      typedef int32_t _order_type;
      _order_type order;

    FibonacciGoal():
      order(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_order;
      u_order.real = this->order;
      *(outbuffer + offset + 0) = (u_order.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_order.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_order.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_order.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->order);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_order;
      u_order.base = 0;
      u_order.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_order.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_order.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_order.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->order = u_order.real;
      offset += sizeof(this->order);
     return offset;
    }

    virtual const char * getType() override { return "actionlib_tutorials/FibonacciGoal"; };
    virtual const char * getMD5() override { return "6889063349a00b249bd1661df429d822"; };

  };

}
#endif
