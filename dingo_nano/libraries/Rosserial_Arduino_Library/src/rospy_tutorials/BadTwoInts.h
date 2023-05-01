#ifndef _ROS_SERVICE_BadTwoInts_h
#define _ROS_SERVICE_BadTwoInts_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"

namespace rospy_tutorials
{

#ifdef ESP8266
    static const char BADTWOINTS[] = "rospy_tutorials/BadTwoInts";
#else
    static const char BADTWOINTS[] PROGMEM = "rospy_tutorials/BadTwoInts";
#endif

  class BadTwoIntsRequest : public ros::Msg
  {
    public:
      typedef int64_t _a_type;
      _a_type a;
      typedef int32_t _b_type;
      _b_type b;

    BadTwoIntsRequest():
      a(0),
      b(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_a;
      u_a.real = this->a;
      *(outbuffer + offset + 0) = (u_a.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_a.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_a.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_a.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_a.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_a.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_a.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_a.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->a);
      union {
        int32_t real;
        uint32_t base;
      } u_b;
      u_b.real = this->b;
      *(outbuffer + offset + 0) = (u_b.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_b.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_b.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_b.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->b);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_a;
      u_a.base = 0;
      u_a.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_a.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_a.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_a.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_a.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_a.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_a.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_a.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->a = u_a.real;
      offset += sizeof(this->a);
      union {
        int32_t real;
        uint32_t base;
      } u_b;
      u_b.base = 0;
      u_b.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_b.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_b.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_b.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->b = u_b.real;
      offset += sizeof(this->b);
     return offset;
    }

    const char * getType(){ return BADTWOINTS; };
    #ifdef ESP8266
        const char * getMD5() { return  ("29bb5c7dea8bf822f53e94b0ee5a3a56");};
    #else
        const char * getMD5() { return  PSTR("29bb5c7dea8bf822f53e94b0ee5a3a56");};
    #endif

  };

  class BadTwoIntsResponse : public ros::Msg
  {
    public:
      typedef int32_t _sum_type;
      _sum_type sum;

    BadTwoIntsResponse():
      sum(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_sum;
      u_sum.real = this->sum;
      *(outbuffer + offset + 0) = (u_sum.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sum.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sum.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sum.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sum);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_sum;
      u_sum.base = 0;
      u_sum.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sum.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sum.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sum.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sum = u_sum.real;
      offset += sizeof(this->sum);
     return offset;
    }

    const char * getType(){ return BADTWOINTS; };
    #ifdef ESP8266
        const char * getMD5() { return  ("0ba699c25c9418c0366f3595c0c8e8ec");};
    #else
        const char * getMD5() { return  PSTR("0ba699c25c9418c0366f3595c0c8e8ec");};
    #endif

  };

  class BadTwoInts {
    public:
    typedef BadTwoIntsRequest Request;
    typedef BadTwoIntsResponse Response;
  };

}
#endif
