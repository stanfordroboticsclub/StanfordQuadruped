#ifndef _ROS_actionlib_TestRequestGoal_h
#define _ROS_actionlib_TestRequestGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/duration.h"

namespace actionlib
{

  class TestRequestGoal : public ros::Msg
  {
    public:
      typedef int32_t _terminate_status_type;
      _terminate_status_type terminate_status;
      typedef bool _ignore_cancel_type;
      _ignore_cancel_type ignore_cancel;
      typedef const char* _result_text_type;
      _result_text_type result_text;
      typedef int32_t _the_result_type;
      _the_result_type the_result;
      typedef bool _is_simple_client_type;
      _is_simple_client_type is_simple_client;
      typedef ros::Duration _delay_accept_type;
      _delay_accept_type delay_accept;
      typedef ros::Duration _delay_terminate_type;
      _delay_terminate_type delay_terminate;
      typedef ros::Duration _pause_status_type;
      _pause_status_type pause_status;
      enum { TERMINATE_SUCCESS =  0 };
      enum { TERMINATE_ABORTED =  1 };
      enum { TERMINATE_REJECTED =  2 };
      enum { TERMINATE_LOSE =  3 };
      enum { TERMINATE_DROP =  4 };
      enum { TERMINATE_EXCEPTION =  5 };

    TestRequestGoal():
      terminate_status(0),
      ignore_cancel(0),
      result_text(""),
      the_result(0),
      is_simple_client(0),
      delay_accept(),
      delay_terminate(),
      pause_status()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_terminate_status;
      u_terminate_status.real = this->terminate_status;
      *(outbuffer + offset + 0) = (u_terminate_status.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_terminate_status.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_terminate_status.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_terminate_status.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->terminate_status);
      union {
        bool real;
        uint8_t base;
      } u_ignore_cancel;
      u_ignore_cancel.real = this->ignore_cancel;
      *(outbuffer + offset + 0) = (u_ignore_cancel.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ignore_cancel);
      uint32_t length_result_text = strlen(this->result_text);
      varToArr(outbuffer + offset, length_result_text);
      offset += 4;
      memcpy(outbuffer + offset, this->result_text, length_result_text);
      offset += length_result_text;
      union {
        int32_t real;
        uint32_t base;
      } u_the_result;
      u_the_result.real = this->the_result;
      *(outbuffer + offset + 0) = (u_the_result.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_the_result.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_the_result.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_the_result.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->the_result);
      union {
        bool real;
        uint8_t base;
      } u_is_simple_client;
      u_is_simple_client.real = this->is_simple_client;
      *(outbuffer + offset + 0) = (u_is_simple_client.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_simple_client);
      *(outbuffer + offset + 0) = (this->delay_accept.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->delay_accept.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->delay_accept.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->delay_accept.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->delay_accept.sec);
      *(outbuffer + offset + 0) = (this->delay_accept.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->delay_accept.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->delay_accept.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->delay_accept.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->delay_accept.nsec);
      *(outbuffer + offset + 0) = (this->delay_terminate.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->delay_terminate.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->delay_terminate.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->delay_terminate.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->delay_terminate.sec);
      *(outbuffer + offset + 0) = (this->delay_terminate.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->delay_terminate.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->delay_terminate.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->delay_terminate.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->delay_terminate.nsec);
      *(outbuffer + offset + 0) = (this->pause_status.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pause_status.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pause_status.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pause_status.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pause_status.sec);
      *(outbuffer + offset + 0) = (this->pause_status.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pause_status.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pause_status.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pause_status.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pause_status.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_terminate_status;
      u_terminate_status.base = 0;
      u_terminate_status.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_terminate_status.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_terminate_status.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_terminate_status.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->terminate_status = u_terminate_status.real;
      offset += sizeof(this->terminate_status);
      union {
        bool real;
        uint8_t base;
      } u_ignore_cancel;
      u_ignore_cancel.base = 0;
      u_ignore_cancel.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ignore_cancel = u_ignore_cancel.real;
      offset += sizeof(this->ignore_cancel);
      uint32_t length_result_text;
      arrToVar(length_result_text, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_result_text; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_result_text-1]=0;
      this->result_text = (char *)(inbuffer + offset-1);
      offset += length_result_text;
      union {
        int32_t real;
        uint32_t base;
      } u_the_result;
      u_the_result.base = 0;
      u_the_result.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_the_result.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_the_result.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_the_result.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->the_result = u_the_result.real;
      offset += sizeof(this->the_result);
      union {
        bool real;
        uint8_t base;
      } u_is_simple_client;
      u_is_simple_client.base = 0;
      u_is_simple_client.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_simple_client = u_is_simple_client.real;
      offset += sizeof(this->is_simple_client);
      this->delay_accept.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->delay_accept.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->delay_accept.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->delay_accept.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->delay_accept.sec);
      this->delay_accept.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->delay_accept.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->delay_accept.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->delay_accept.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->delay_accept.nsec);
      this->delay_terminate.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->delay_terminate.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->delay_terminate.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->delay_terminate.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->delay_terminate.sec);
      this->delay_terminate.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->delay_terminate.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->delay_terminate.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->delay_terminate.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->delay_terminate.nsec);
      this->pause_status.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->pause_status.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pause_status.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->pause_status.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->pause_status.sec);
      this->pause_status.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->pause_status.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pause_status.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->pause_status.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->pause_status.nsec);
     return offset;
    }

    virtual const char * getType() override { return "actionlib/TestRequestGoal"; };
    virtual const char * getMD5() override { return "db5d00ba98302d6c6dd3737e9a03ceea"; };

  };

}
#endif
