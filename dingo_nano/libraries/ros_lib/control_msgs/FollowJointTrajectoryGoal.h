#ifndef _ROS_control_msgs_FollowJointTrajectoryGoal_h
#define _ROS_control_msgs_FollowJointTrajectoryGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "control_msgs/JointTolerance.h"
#include "ros/duration.h"

namespace control_msgs
{

  class FollowJointTrajectoryGoal : public ros::Msg
  {
    public:
      typedef trajectory_msgs::JointTrajectory _trajectory_type;
      _trajectory_type trajectory;
      uint32_t path_tolerance_length;
      typedef control_msgs::JointTolerance _path_tolerance_type;
      _path_tolerance_type st_path_tolerance;
      _path_tolerance_type * path_tolerance;
      uint32_t goal_tolerance_length;
      typedef control_msgs::JointTolerance _goal_tolerance_type;
      _goal_tolerance_type st_goal_tolerance;
      _goal_tolerance_type * goal_tolerance;
      typedef ros::Duration _goal_time_tolerance_type;
      _goal_time_tolerance_type goal_time_tolerance;

    FollowJointTrajectoryGoal():
      trajectory(),
      path_tolerance_length(0), st_path_tolerance(), path_tolerance(nullptr),
      goal_tolerance_length(0), st_goal_tolerance(), goal_tolerance(nullptr),
      goal_time_tolerance()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->trajectory.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->path_tolerance_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->path_tolerance_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->path_tolerance_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->path_tolerance_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->path_tolerance_length);
      for( uint32_t i = 0; i < path_tolerance_length; i++){
      offset += this->path_tolerance[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->goal_tolerance_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->goal_tolerance_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->goal_tolerance_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->goal_tolerance_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->goal_tolerance_length);
      for( uint32_t i = 0; i < goal_tolerance_length; i++){
      offset += this->goal_tolerance[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->goal_time_tolerance.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->goal_time_tolerance.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->goal_time_tolerance.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->goal_time_tolerance.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->goal_time_tolerance.sec);
      *(outbuffer + offset + 0) = (this->goal_time_tolerance.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->goal_time_tolerance.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->goal_time_tolerance.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->goal_time_tolerance.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->goal_time_tolerance.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->trajectory.deserialize(inbuffer + offset);
      uint32_t path_tolerance_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      path_tolerance_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      path_tolerance_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      path_tolerance_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->path_tolerance_length);
      if(path_tolerance_lengthT > path_tolerance_length)
        this->path_tolerance = (control_msgs::JointTolerance*)realloc(this->path_tolerance, path_tolerance_lengthT * sizeof(control_msgs::JointTolerance));
      path_tolerance_length = path_tolerance_lengthT;
      for( uint32_t i = 0; i < path_tolerance_length; i++){
      offset += this->st_path_tolerance.deserialize(inbuffer + offset);
        memcpy( &(this->path_tolerance[i]), &(this->st_path_tolerance), sizeof(control_msgs::JointTolerance));
      }
      uint32_t goal_tolerance_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      goal_tolerance_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      goal_tolerance_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      goal_tolerance_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->goal_tolerance_length);
      if(goal_tolerance_lengthT > goal_tolerance_length)
        this->goal_tolerance = (control_msgs::JointTolerance*)realloc(this->goal_tolerance, goal_tolerance_lengthT * sizeof(control_msgs::JointTolerance));
      goal_tolerance_length = goal_tolerance_lengthT;
      for( uint32_t i = 0; i < goal_tolerance_length; i++){
      offset += this->st_goal_tolerance.deserialize(inbuffer + offset);
        memcpy( &(this->goal_tolerance[i]), &(this->st_goal_tolerance), sizeof(control_msgs::JointTolerance));
      }
      this->goal_time_tolerance.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->goal_time_tolerance.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->goal_time_tolerance.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->goal_time_tolerance.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->goal_time_tolerance.sec);
      this->goal_time_tolerance.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->goal_time_tolerance.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->goal_time_tolerance.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->goal_time_tolerance.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->goal_time_tolerance.nsec);
     return offset;
    }

    virtual const char * getType() override { return "control_msgs/FollowJointTrajectoryGoal"; };
    virtual const char * getMD5() override { return "69636787b6ecbde4d61d711979bc7ecb"; };

  };

}
#endif
