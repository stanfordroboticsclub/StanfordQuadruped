#ifndef _ROS_control_msgs_GripperCommandResult_h
#define _ROS_control_msgs_GripperCommandResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace control_msgs
{

  class GripperCommandResult : public ros::Msg
  {
    public:
      typedef float _position_type;
      _position_type position;
      typedef float _effort_type;
      _effort_type effort;
      typedef bool _stalled_type;
      _stalled_type stalled;
      typedef bool _reached_goal_type;
      _reached_goal_type reached_goal;

    GripperCommandResult():
      position(0),
      effort(0),
      stalled(0),
      reached_goal(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->position);
      offset += serializeAvrFloat64(outbuffer + offset, this->effort);
      union {
        bool real;
        uint8_t base;
      } u_stalled;
      u_stalled.real = this->stalled;
      *(outbuffer + offset + 0) = (u_stalled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->stalled);
      union {
        bool real;
        uint8_t base;
      } u_reached_goal;
      u_reached_goal.real = this->reached_goal;
      *(outbuffer + offset + 0) = (u_reached_goal.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reached_goal);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->position));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->effort));
      union {
        bool real;
        uint8_t base;
      } u_stalled;
      u_stalled.base = 0;
      u_stalled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->stalled = u_stalled.real;
      offset += sizeof(this->stalled);
      union {
        bool real;
        uint8_t base;
      } u_reached_goal;
      u_reached_goal.base = 0;
      u_reached_goal.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->reached_goal = u_reached_goal.real;
      offset += sizeof(this->reached_goal);
     return offset;
    }

    virtual const char * getType() override { return "control_msgs/GripperCommandResult"; };
    virtual const char * getMD5() override { return "e4cbff56d3562bcf113da5a5adeef91f"; };

  };

}
#endif
