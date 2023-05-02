#ifndef _ROS_actionlib_TestRequestResult_h
#define _ROS_actionlib_TestRequestResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace actionlib
{

  class TestRequestResult : public ros::Msg
  {
    public:
      typedef int32_t _the_result_type;
      _the_result_type the_result;
      typedef bool _is_simple_server_type;
      _is_simple_server_type is_simple_server;

    TestRequestResult():
      the_result(0),
      is_simple_server(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
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
      } u_is_simple_server;
      u_is_simple_server.real = this->is_simple_server;
      *(outbuffer + offset + 0) = (u_is_simple_server.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_simple_server);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
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
      } u_is_simple_server;
      u_is_simple_server.base = 0;
      u_is_simple_server.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_simple_server = u_is_simple_server.real;
      offset += sizeof(this->is_simple_server);
     return offset;
    }

    virtual const char * getType() override { return "actionlib/TestRequestResult"; };
    virtual const char * getMD5() override { return "61c2364524499c7c5017e2f3fce7ba06"; };

  };

}
#endif
