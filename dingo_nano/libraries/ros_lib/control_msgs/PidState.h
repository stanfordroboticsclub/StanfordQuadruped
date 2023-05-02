#ifndef _ROS_control_msgs_PidState_h
#define _ROS_control_msgs_PidState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ros/duration.h"

namespace control_msgs
{

  class PidState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef ros::Duration _timestep_type;
      _timestep_type timestep;
      typedef float _error_type;
      _error_type error;
      typedef float _error_dot_type;
      _error_dot_type error_dot;
      typedef float _p_error_type;
      _p_error_type p_error;
      typedef float _i_error_type;
      _i_error_type i_error;
      typedef float _d_error_type;
      _d_error_type d_error;
      typedef float _p_term_type;
      _p_term_type p_term;
      typedef float _i_term_type;
      _i_term_type i_term;
      typedef float _d_term_type;
      _d_term_type d_term;
      typedef float _i_max_type;
      _i_max_type i_max;
      typedef float _i_min_type;
      _i_min_type i_min;
      typedef float _output_type;
      _output_type output;

    PidState():
      header(),
      timestep(),
      error(0),
      error_dot(0),
      p_error(0),
      i_error(0),
      d_error(0),
      p_term(0),
      i_term(0),
      d_term(0),
      i_max(0),
      i_min(0),
      output(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->timestep.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestep.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestep.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestep.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestep.sec);
      *(outbuffer + offset + 0) = (this->timestep.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestep.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestep.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestep.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestep.nsec);
      offset += serializeAvrFloat64(outbuffer + offset, this->error);
      offset += serializeAvrFloat64(outbuffer + offset, this->error_dot);
      offset += serializeAvrFloat64(outbuffer + offset, this->p_error);
      offset += serializeAvrFloat64(outbuffer + offset, this->i_error);
      offset += serializeAvrFloat64(outbuffer + offset, this->d_error);
      offset += serializeAvrFloat64(outbuffer + offset, this->p_term);
      offset += serializeAvrFloat64(outbuffer + offset, this->i_term);
      offset += serializeAvrFloat64(outbuffer + offset, this->d_term);
      offset += serializeAvrFloat64(outbuffer + offset, this->i_max);
      offset += serializeAvrFloat64(outbuffer + offset, this->i_min);
      offset += serializeAvrFloat64(outbuffer + offset, this->output);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->timestep.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestep.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestep.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestep.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestep.sec);
      this->timestep.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestep.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestep.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestep.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestep.nsec);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->error));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->error_dot));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->p_error));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->i_error));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->d_error));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->p_term));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->i_term));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->d_term));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->i_max));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->i_min));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->output));
     return offset;
    }

    virtual const char * getType() override { return "control_msgs/PidState"; };
    virtual const char * getMD5() override { return "b138ec00e886c10e73f27e8712252ea6"; };

  };

}
#endif
