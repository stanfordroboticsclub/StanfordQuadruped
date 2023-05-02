#ifndef _ROS_turtlesim_Pose_h
#define _ROS_turtlesim_Pose_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace turtlesim
{

  class Pose : public ros::Msg
  {
    public:
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _theta_type;
      _theta_type theta;
      typedef float _linear_velocity_type;
      _linear_velocity_type linear_velocity;
      typedef float _angular_velocity_type;
      _angular_velocity_type angular_velocity;

    Pose():
      x(0),
      y(0),
      theta(0),
      linear_velocity(0),
      angular_velocity(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_theta;
      u_theta.real = this->theta;
      *(outbuffer + offset + 0) = (u_theta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_theta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_theta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->theta);
      union {
        float real;
        uint32_t base;
      } u_linear_velocity;
      u_linear_velocity.real = this->linear_velocity;
      *(outbuffer + offset + 0) = (u_linear_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_linear_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_linear_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_linear_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->linear_velocity);
      union {
        float real;
        uint32_t base;
      } u_angular_velocity;
      u_angular_velocity.real = this->angular_velocity;
      *(outbuffer + offset + 0) = (u_angular_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angular_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angular_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angular_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angular_velocity);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_theta;
      u_theta.base = 0;
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->theta = u_theta.real;
      offset += sizeof(this->theta);
      union {
        float real;
        uint32_t base;
      } u_linear_velocity;
      u_linear_velocity.base = 0;
      u_linear_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_linear_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_linear_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_linear_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->linear_velocity = u_linear_velocity.real;
      offset += sizeof(this->linear_velocity);
      union {
        float real;
        uint32_t base;
      } u_angular_velocity;
      u_angular_velocity.base = 0;
      u_angular_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angular_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angular_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angular_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angular_velocity = u_angular_velocity.real;
      offset += sizeof(this->angular_velocity);
     return offset;
    }

    virtual const char * getType() override { return "turtlesim/Pose"; };
    virtual const char * getMD5() override { return "863b248d5016ca62ea2e895ae5265cf9"; };

  };

}
#endif
