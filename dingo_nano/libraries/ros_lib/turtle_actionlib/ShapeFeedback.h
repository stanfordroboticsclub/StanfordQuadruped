#ifndef _ROS_turtle_actionlib_ShapeFeedback_h
#define _ROS_turtle_actionlib_ShapeFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace turtle_actionlib
{

  class ShapeFeedback : public ros::Msg
  {
    public:

    ShapeFeedback()
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

    virtual const char * getType() override { return "turtle_actionlib/ShapeFeedback"; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif
