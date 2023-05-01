#ifndef _ROS_actionlib_TestGoal_h
#define _ROS_actionlib_TestGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace actionlib
{

  class TestGoal : public ros::Msg
  {
    public:
      typedef int32_t _goal_type;
      _goal_type goal;

    TestGoal():
      goal(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_goal;
      u_goal.real = this->goal;
      *(outbuffer + offset + 0) = (u_goal.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_goal.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_goal.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_goal.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->goal);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_goal;
      u_goal.base = 0;
      u_goal.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_goal.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_goal.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_goal.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->goal = u_goal.real;
      offset += sizeof(this->goal);
     return offset;
    }

    virtual const char * getType() override { return "actionlib/TestGoal"; };
    virtual const char * getMD5() override { return "18df0149936b7aa95588e3862476ebde"; };

  };

}
#endif
