#ifndef _ROS_control_msgs_GripperCommand_h
#define _ROS_control_msgs_GripperCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace control_msgs
{

  class GripperCommand : public ros::Msg
  {
    public:
      typedef float _position_type;
      _position_type position;
      typedef float _max_effort_type;
      _max_effort_type max_effort;

    GripperCommand():
      position(0),
      max_effort(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->position);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_effort);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->position));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_effort));
     return offset;
    }

    virtual const char * getType() override { return "control_msgs/GripperCommand"; };
    virtual const char * getMD5() override { return "680acaff79486f017132a7f198d40f08"; };

  };

}
#endif
