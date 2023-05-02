#ifndef _ROS_actionlib_tutorials_FibonacciResult_h
#define _ROS_actionlib_tutorials_FibonacciResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace actionlib_tutorials
{

  class FibonacciResult : public ros::Msg
  {
    public:
      uint32_t sequence_length;
      typedef int32_t _sequence_type;
      _sequence_type st_sequence;
      _sequence_type * sequence;

    FibonacciResult():
      sequence_length(0), st_sequence(), sequence(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->sequence_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sequence_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sequence_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sequence_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sequence_length);
      for( uint32_t i = 0; i < sequence_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_sequencei;
      u_sequencei.real = this->sequence[i];
      *(outbuffer + offset + 0) = (u_sequencei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sequencei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sequencei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sequencei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sequence[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t sequence_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sequence_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sequence_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sequence_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sequence_length);
      if(sequence_lengthT > sequence_length)
        this->sequence = (int32_t*)realloc(this->sequence, sequence_lengthT * sizeof(int32_t));
      sequence_length = sequence_lengthT;
      for( uint32_t i = 0; i < sequence_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_sequence;
      u_st_sequence.base = 0;
      u_st_sequence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_sequence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_sequence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_sequence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_sequence = u_st_sequence.real;
      offset += sizeof(this->st_sequence);
        memcpy( &(this->sequence[i]), &(this->st_sequence), sizeof(int32_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "actionlib_tutorials/FibonacciResult"; };
    virtual const char * getMD5() override { return "b81e37d2a31925a0e8ae261a8699cb79"; };

  };

}
#endif
