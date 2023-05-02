#ifndef _ROS_control_msgs_SingleJointPositionGoal_h
#define _ROS_control_msgs_SingleJointPositionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/duration.h"

namespace control_msgs
{

  class SingleJointPositionGoal : public ros::Msg
  {
    public:
      typedef float _position_type;
      _position_type position;
      typedef ros::Duration _min_duration_type;
      _min_duration_type min_duration;
      typedef float _max_velocity_type;
      _max_velocity_type max_velocity;

    SingleJointPositionGoal():
      position(0),
      min_duration(),
      max_velocity(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->position);
      *(outbuffer + offset + 0) = (this->min_duration.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->min_duration.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->min_duration.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->min_duration.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_duration.sec);
      *(outbuffer + offset + 0) = (this->min_duration.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->min_duration.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->min_duration.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->min_duration.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_duration.nsec);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_velocity);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->position));
      this->min_duration.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->min_duration.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->min_duration.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->min_duration.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->min_duration.sec);
      this->min_duration.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->min_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->min_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->min_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->min_duration.nsec);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_velocity));
     return offset;
    }

    virtual const char * getType() override { return "control_msgs/SingleJointPositionGoal"; };
    virtual const char * getMD5() override { return "fbaaa562a23a013fd5053e5f72cbb35c"; };

  };

}
#endif
