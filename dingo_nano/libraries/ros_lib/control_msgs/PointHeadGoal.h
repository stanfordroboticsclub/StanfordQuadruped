#ifndef _ROS_control_msgs_PointHeadGoal_h
#define _ROS_control_msgs_PointHeadGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3.h"
#include "ros/duration.h"

namespace control_msgs
{

  class PointHeadGoal : public ros::Msg
  {
    public:
      typedef geometry_msgs::PointStamped _target_type;
      _target_type target;
      typedef geometry_msgs::Vector3 _pointing_axis_type;
      _pointing_axis_type pointing_axis;
      typedef const char* _pointing_frame_type;
      _pointing_frame_type pointing_frame;
      typedef ros::Duration _min_duration_type;
      _min_duration_type min_duration;
      typedef float _max_velocity_type;
      _max_velocity_type max_velocity;

    PointHeadGoal():
      target(),
      pointing_axis(),
      pointing_frame(""),
      min_duration(),
      max_velocity(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->target.serialize(outbuffer + offset);
      offset += this->pointing_axis.serialize(outbuffer + offset);
      uint32_t length_pointing_frame = strlen(this->pointing_frame);
      varToArr(outbuffer + offset, length_pointing_frame);
      offset += 4;
      memcpy(outbuffer + offset, this->pointing_frame, length_pointing_frame);
      offset += length_pointing_frame;
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
      offset += this->target.deserialize(inbuffer + offset);
      offset += this->pointing_axis.deserialize(inbuffer + offset);
      uint32_t length_pointing_frame;
      arrToVar(length_pointing_frame, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_pointing_frame; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_pointing_frame-1]=0;
      this->pointing_frame = (char *)(inbuffer + offset-1);
      offset += length_pointing_frame;
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

    virtual const char * getType() override { return "control_msgs/PointHeadGoal"; };
    virtual const char * getMD5() override { return "8b92b1cd5e06c8a94c917dc3209a4c1d"; };

  };

}
#endif
