#ifndef _ROS_dynamic_reconfigure_SensorLevels_h
#define _ROS_dynamic_reconfigure_SensorLevels_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamic_reconfigure
{

  class SensorLevels : public ros::Msg
  {
    public:
      enum { RECONFIGURE_CLOSE =  3   };
      enum { RECONFIGURE_STOP =  1   };
      enum { RECONFIGURE_RUNNING =  0  };

    SensorLevels()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return "dynamic_reconfigure/SensorLevels"; };
    virtual const char * getMD5() override { return "6322637bee96d5489db6e2127c47602c"; };

  };

}
#endif
