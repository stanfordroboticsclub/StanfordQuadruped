#ifndef _ROS_tf2_msgs_LookupTransformGoal_h
#define _ROS_tf2_msgs_LookupTransformGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "ros/duration.h"

namespace tf2_msgs
{

  class LookupTransformGoal : public ros::Msg
  {
    public:
      typedef const char* _target_frame_type;
      _target_frame_type target_frame;
      typedef const char* _source_frame_type;
      _source_frame_type source_frame;
      typedef ros::Time _source_time_type;
      _source_time_type source_time;
      typedef ros::Duration _timeout_type;
      _timeout_type timeout;
      typedef ros::Time _target_time_type;
      _target_time_type target_time;
      typedef const char* _fixed_frame_type;
      _fixed_frame_type fixed_frame;
      typedef bool _advanced_type;
      _advanced_type advanced;

    LookupTransformGoal():
      target_frame(""),
      source_frame(""),
      source_time(),
      timeout(),
      target_time(),
      fixed_frame(""),
      advanced(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_target_frame = strlen(this->target_frame);
      varToArr(outbuffer + offset, length_target_frame);
      offset += 4;
      memcpy(outbuffer + offset, this->target_frame, length_target_frame);
      offset += length_target_frame;
      uint32_t length_source_frame = strlen(this->source_frame);
      varToArr(outbuffer + offset, length_source_frame);
      offset += 4;
      memcpy(outbuffer + offset, this->source_frame, length_source_frame);
      offset += length_source_frame;
      *(outbuffer + offset + 0) = (this->source_time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->source_time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->source_time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->source_time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->source_time.sec);
      *(outbuffer + offset + 0) = (this->source_time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->source_time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->source_time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->source_time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->source_time.nsec);
      *(outbuffer + offset + 0) = (this->timeout.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timeout.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timeout.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timeout.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timeout.sec);
      *(outbuffer + offset + 0) = (this->timeout.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timeout.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timeout.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timeout.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timeout.nsec);
      *(outbuffer + offset + 0) = (this->target_time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->target_time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->target_time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->target_time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_time.sec);
      *(outbuffer + offset + 0) = (this->target_time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->target_time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->target_time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->target_time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_time.nsec);
      uint32_t length_fixed_frame = strlen(this->fixed_frame);
      varToArr(outbuffer + offset, length_fixed_frame);
      offset += 4;
      memcpy(outbuffer + offset, this->fixed_frame, length_fixed_frame);
      offset += length_fixed_frame;
      union {
        bool real;
        uint8_t base;
      } u_advanced;
      u_advanced.real = this->advanced;
      *(outbuffer + offset + 0) = (u_advanced.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->advanced);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_target_frame;
      arrToVar(length_target_frame, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_target_frame; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_target_frame-1]=0;
      this->target_frame = (char *)(inbuffer + offset-1);
      offset += length_target_frame;
      uint32_t length_source_frame;
      arrToVar(length_source_frame, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_source_frame; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_source_frame-1]=0;
      this->source_frame = (char *)(inbuffer + offset-1);
      offset += length_source_frame;
      this->source_time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->source_time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->source_time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->source_time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->source_time.sec);
      this->source_time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->source_time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->source_time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->source_time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->source_time.nsec);
      this->timeout.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->timeout.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timeout.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timeout.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timeout.sec);
      this->timeout.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->timeout.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timeout.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timeout.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timeout.nsec);
      this->target_time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->target_time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->target_time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->target_time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->target_time.sec);
      this->target_time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->target_time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->target_time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->target_time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->target_time.nsec);
      uint32_t length_fixed_frame;
      arrToVar(length_fixed_frame, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_fixed_frame; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_fixed_frame-1]=0;
      this->fixed_frame = (char *)(inbuffer + offset-1);
      offset += length_fixed_frame;
      union {
        bool real;
        uint8_t base;
      } u_advanced;
      u_advanced.base = 0;
      u_advanced.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->advanced = u_advanced.real;
      offset += sizeof(this->advanced);
     return offset;
    }

    virtual const char * getType() override { return "tf2_msgs/LookupTransformGoal"; };
    virtual const char * getMD5() override { return "35e3720468131d675a18bb6f3e5f22f8"; };

  };

}
#endif
