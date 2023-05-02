#ifndef _ROS_SERVICE_SetJointTrajectory_h
#define _ROS_SERVICE_SetJointTrajectory_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "trajectory_msgs/JointTrajectory.h"

namespace gazebo_msgs
{

static const char SETJOINTTRAJECTORY[] = "gazebo_msgs/SetJointTrajectory";

  class SetJointTrajectoryRequest : public ros::Msg
  {
    public:
      typedef const char* _model_name_type;
      _model_name_type model_name;
      typedef trajectory_msgs::JointTrajectory _joint_trajectory_type;
      _joint_trajectory_type joint_trajectory;
      typedef geometry_msgs::Pose _model_pose_type;
      _model_pose_type model_pose;
      typedef bool _set_model_pose_type;
      _set_model_pose_type set_model_pose;
      typedef bool _disable_physics_updates_type;
      _disable_physics_updates_type disable_physics_updates;

    SetJointTrajectoryRequest():
      model_name(""),
      joint_trajectory(),
      model_pose(),
      set_model_pose(0),
      disable_physics_updates(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_model_name = strlen(this->model_name);
      varToArr(outbuffer + offset, length_model_name);
      offset += 4;
      memcpy(outbuffer + offset, this->model_name, length_model_name);
      offset += length_model_name;
      offset += this->joint_trajectory.serialize(outbuffer + offset);
      offset += this->model_pose.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_set_model_pose;
      u_set_model_pose.real = this->set_model_pose;
      *(outbuffer + offset + 0) = (u_set_model_pose.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->set_model_pose);
      union {
        bool real;
        uint8_t base;
      } u_disable_physics_updates;
      u_disable_physics_updates.real = this->disable_physics_updates;
      *(outbuffer + offset + 0) = (u_disable_physics_updates.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->disable_physics_updates);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_model_name;
      arrToVar(length_model_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_model_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_model_name-1]=0;
      this->model_name = (char *)(inbuffer + offset-1);
      offset += length_model_name;
      offset += this->joint_trajectory.deserialize(inbuffer + offset);
      offset += this->model_pose.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_set_model_pose;
      u_set_model_pose.base = 0;
      u_set_model_pose.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->set_model_pose = u_set_model_pose.real;
      offset += sizeof(this->set_model_pose);
      union {
        bool real;
        uint8_t base;
      } u_disable_physics_updates;
      u_disable_physics_updates.base = 0;
      u_disable_physics_updates.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->disable_physics_updates = u_disable_physics_updates.real;
      offset += sizeof(this->disable_physics_updates);
     return offset;
    }

    virtual const char * getType() override { return SETJOINTTRAJECTORY; };
    virtual const char * getMD5() override { return "649dd2eba5ffd358069238825f9f85ab"; };

  };

  class SetJointTrajectoryResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _status_message_type;
      _status_message_type status_message;

    SetJointTrajectoryResponse():
      success(0),
      status_message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_status_message = strlen(this->status_message);
      varToArr(outbuffer + offset, length_status_message);
      offset += 4;
      memcpy(outbuffer + offset, this->status_message, length_status_message);
      offset += length_status_message;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_status_message;
      arrToVar(length_status_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_status_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_status_message-1]=0;
      this->status_message = (char *)(inbuffer + offset-1);
      offset += length_status_message;
     return offset;
    }

    virtual const char * getType() override { return SETJOINTTRAJECTORY; };
    virtual const char * getMD5() override { return "2ec6f3eff0161f4257b808b12bc830c2"; };

  };

  class SetJointTrajectory {
    public:
    typedef SetJointTrajectoryRequest Request;
    typedef SetJointTrajectoryResponse Response;
  };

}
#endif
