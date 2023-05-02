#ifndef _ROS_control_msgs_GripperCommandGoal_h
#define _ROS_control_msgs_GripperCommandGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "control_msgs/GripperCommand.h"

namespace control_msgs
{

  class GripperCommandGoal : public ros::Msg
  {
    public:
      typedef control_msgs::GripperCommand _command_type;
      _command_type command;

    GripperCommandGoal():
      command()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->command.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->command.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "control_msgs/GripperCommandGoal"; };
    virtual const char * getMD5() override { return "86fd82f4ddc48a4cb6856cfa69217e43"; };

  };

}
#endif
