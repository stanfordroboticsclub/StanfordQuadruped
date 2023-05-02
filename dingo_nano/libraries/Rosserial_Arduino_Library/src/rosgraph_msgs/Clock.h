#ifndef _ROS_rosgraph_msgs_Clock_h
#define _ROS_rosgraph_msgs_Clock_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace rosgraph_msgs
{

  class Clock : public ros::Msg
  {
    public:
      typedef ros::Time _clock_type;
      _clock_type clock;

    Clock():
      clock()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->clock.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->clock.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->clock.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->clock.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->clock.sec);
      *(outbuffer + offset + 0) = (this->clock.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->clock.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->clock.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->clock.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->clock.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->clock.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->clock.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->clock.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->clock.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->clock.sec);
      this->clock.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->clock.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->clock.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->clock.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->clock.nsec);
     return offset;
    }

    virtual const char * getType() override { return "rosgraph_msgs/Clock"; };
    virtual const char * getMD5() override { return "a9c97c1d230cfc112e270351a944ee47"; };

  };

}
#endif
