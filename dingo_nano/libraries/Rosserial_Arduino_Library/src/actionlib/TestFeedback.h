#ifndef _ROS_actionlib_TestFeedback_h
#define _ROS_actionlib_TestFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace actionlib
{

  class TestFeedback : public ros::Msg
  {
    public:
      typedef int32_t _feedback_type;
      _feedback_type feedback;

    TestFeedback():
      feedback(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_feedback;
      u_feedback.real = this->feedback;
      *(outbuffer + offset + 0) = (u_feedback.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_feedback.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_feedback.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_feedback.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->feedback);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_feedback;
      u_feedback.base = 0;
      u_feedback.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_feedback.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_feedback.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_feedback.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->feedback = u_feedback.real;
      offset += sizeof(this->feedback);
     return offset;
    }

    virtual const char * getType() override { return "actionlib/TestFeedback"; };
    virtual const char * getMD5() override { return "49ceb5b32ea3af22073ede4a0328249e"; };

  };

}
#endif
