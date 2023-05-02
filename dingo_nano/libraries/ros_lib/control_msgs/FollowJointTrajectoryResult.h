#ifndef _ROS_control_msgs_FollowJointTrajectoryResult_h
#define _ROS_control_msgs_FollowJointTrajectoryResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace control_msgs
{

  class FollowJointTrajectoryResult : public ros::Msg
  {
    public:
      typedef int32_t _error_code_type;
      _error_code_type error_code;
      typedef const char* _error_string_type;
      _error_string_type error_string;
      enum { SUCCESSFUL =  0 };
      enum { INVALID_GOAL =  -1 };
      enum { INVALID_JOINTS =  -2 };
      enum { OLD_HEADER_TIMESTAMP =  -3 };
      enum { PATH_TOLERANCE_VIOLATED =  -4 };
      enum { GOAL_TOLERANCE_VIOLATED =  -5 };

    FollowJointTrajectoryResult():
      error_code(0),
      error_string("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_error_code;
      u_error_code.real = this->error_code;
      *(outbuffer + offset + 0) = (u_error_code.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_error_code.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_error_code.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_error_code.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->error_code);
      uint32_t length_error_string = strlen(this->error_string);
      varToArr(outbuffer + offset, length_error_string);
      offset += 4;
      memcpy(outbuffer + offset, this->error_string, length_error_string);
      offset += length_error_string;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_error_code;
      u_error_code.base = 0;
      u_error_code.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_error_code.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_error_code.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_error_code.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->error_code = u_error_code.real;
      offset += sizeof(this->error_code);
      uint32_t length_error_string;
      arrToVar(length_error_string, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_error_string; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_error_string-1]=0;
      this->error_string = (char *)(inbuffer + offset-1);
      offset += length_error_string;
     return offset;
    }

    virtual const char * getType() override { return "control_msgs/FollowJointTrajectoryResult"; };
    virtual const char * getMD5() override { return "493383b18409bfb604b4e26c676401d2"; };

  };

}
#endif
