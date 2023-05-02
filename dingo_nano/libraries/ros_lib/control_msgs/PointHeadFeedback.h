#ifndef _ROS_control_msgs_PointHeadFeedback_h
#define _ROS_control_msgs_PointHeadFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace control_msgs
{

  class PointHeadFeedback : public ros::Msg
  {
    public:
      typedef float _pointing_angle_error_type;
      _pointing_angle_error_type pointing_angle_error;

    PointHeadFeedback():
      pointing_angle_error(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->pointing_angle_error);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pointing_angle_error));
     return offset;
    }

    virtual const char * getType() override { return "control_msgs/PointHeadFeedback"; };
    virtual const char * getMD5() override { return "cce80d27fd763682da8805a73316cab4"; };

  };

}
#endif
