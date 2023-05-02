#ifndef _ROS_SERVICE_AssembleScans_h
#define _ROS_SERVICE_AssembleScans_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"
#include "ros/time.h"
#include "sensor_msgs/PointCloud.h"

namespace laser_assembler
{

#ifdef ESP8266
    static const char ASSEMBLESCANS[] = "laser_assembler/AssembleScans";
#else
    static const char ASSEMBLESCANS[] PROGMEM = "laser_assembler/AssembleScans";
#endif

  class AssembleScansRequest : public ros::Msg
  {
    public:
      typedef ros::Time _begin_type;
      _begin_type begin;
      typedef ros::Time _end_type;
      _end_type end;

    AssembleScansRequest():
      begin(),
      end()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->begin.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->begin.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->begin.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->begin.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->begin.sec);
      *(outbuffer + offset + 0) = (this->begin.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->begin.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->begin.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->begin.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->begin.nsec);
      *(outbuffer + offset + 0) = (this->end.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->end.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->end.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->end.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->end.sec);
      *(outbuffer + offset + 0) = (this->end.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->end.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->end.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->end.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->end.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->begin.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->begin.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->begin.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->begin.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->begin.sec);
      this->begin.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->begin.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->begin.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->begin.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->begin.nsec);
      this->end.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->end.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->end.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->end.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->end.sec);
      this->end.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->end.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->end.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->end.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->end.nsec);
     return offset;
    }

    const char * getType(){ return ASSEMBLESCANS; };
    #ifdef ESP8266
        const char * getMD5() { return  ("b341004f74e15bf5e1b2053a9183bdc7");};
    #else
        const char * getMD5() { return  PSTR("b341004f74e15bf5e1b2053a9183bdc7");};
    #endif

  };

  class AssembleScansResponse : public ros::Msg
  {
    public:
      typedef sensor_msgs::PointCloud _cloud_type;
      _cloud_type cloud;

    AssembleScansResponse():
      cloud()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->cloud.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->cloud.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return ASSEMBLESCANS; };
    #ifdef ESP8266
        const char * getMD5() { return  ("4217b28a903e4ad7869a83b3653110ff");};
    #else
        const char * getMD5() { return  PSTR("4217b28a903e4ad7869a83b3653110ff");};
    #endif

  };

  class AssembleScans {
    public:
    typedef AssembleScansRequest Request;
    typedef AssembleScansResponse Response;
  };

}
#endif
