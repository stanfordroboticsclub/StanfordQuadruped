#ifndef _ROS_control_msgs_JointTrajectoryFeedback_h
#define _ROS_control_msgs_JointTrajectoryFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace control_msgs
{

  class JointTrajectoryFeedback : public ros::Msg
  {
    public:

    JointTrajectoryFeedback()
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

    virtual const char * getType() override { return "control_msgs/JointTrajectoryFeedback"; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif
