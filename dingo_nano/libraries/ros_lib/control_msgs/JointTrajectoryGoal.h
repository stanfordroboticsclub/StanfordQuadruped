#ifndef _ROS_control_msgs_JointTrajectoryGoal_h
#define _ROS_control_msgs_JointTrajectoryGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "trajectory_msgs/JointTrajectory.h"

namespace control_msgs
{

  class JointTrajectoryGoal : public ros::Msg
  {
    public:
      typedef trajectory_msgs::JointTrajectory _trajectory_type;
      _trajectory_type trajectory;

    JointTrajectoryGoal():
      trajectory()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->trajectory.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->trajectory.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "control_msgs/JointTrajectoryGoal"; };
    virtual const char * getMD5() override { return "2a0eff76c870e8595636c2a562ca298e"; };

  };

}
#endif
