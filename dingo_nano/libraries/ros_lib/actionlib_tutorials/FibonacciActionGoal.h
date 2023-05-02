#ifndef _ROS_actionlib_tutorials_FibonacciActionGoal_h
#define _ROS_actionlib_tutorials_FibonacciActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "actionlib_tutorials/FibonacciGoal.h"

namespace actionlib_tutorials
{

  class FibonacciActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef actionlib_tutorials::FibonacciGoal _goal_type;
      _goal_type goal;

    FibonacciActionGoal():
      header(),
      goal_id(),
      goal()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->goal_id.serialize(outbuffer + offset);
      offset += this->goal.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->goal_id.deserialize(inbuffer + offset);
      offset += this->goal.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "actionlib_tutorials/FibonacciActionGoal"; };
    virtual const char * getMD5() override { return "006871c7fa1d0e3d5fe2226bf17b2a94"; };

  };

}
#endif
