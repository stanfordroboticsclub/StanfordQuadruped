#ifndef _ROS_SERVICE_QueryCalibrationState_h
#define _ROS_SERVICE_QueryCalibrationState_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"

namespace control_msgs
{

#ifdef ESP8266
    static const char QUERYCALIBRATIONSTATE[] = "control_msgs/QueryCalibrationState";
#else
    static const char QUERYCALIBRATIONSTATE[] PROGMEM = "control_msgs/QueryCalibrationState";
#endif

  class QueryCalibrationStateRequest : public ros::Msg
  {
    public:

    QueryCalibrationStateRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return QUERYCALIBRATIONSTATE; };
    #ifdef ESP8266
        const char * getMD5() { return  ("d41d8cd98f00b204e9800998ecf8427e");};
    #else
        const char * getMD5() { return  PSTR("d41d8cd98f00b204e9800998ecf8427e");};
    #endif

  };

  class QueryCalibrationStateResponse : public ros::Msg
  {
    public:
      typedef bool _is_calibrated_type;
      _is_calibrated_type is_calibrated;

    QueryCalibrationStateResponse():
      is_calibrated(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_calibrated;
      u_is_calibrated.real = this->is_calibrated;
      *(outbuffer + offset + 0) = (u_is_calibrated.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_calibrated);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_calibrated;
      u_is_calibrated.base = 0;
      u_is_calibrated.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_calibrated = u_is_calibrated.real;
      offset += sizeof(this->is_calibrated);
     return offset;
    }

    const char * getType(){ return QUERYCALIBRATIONSTATE; };
    #ifdef ESP8266
        const char * getMD5() { return  ("28af3beedcb84986b8e470dc5470507d");};
    #else
        const char * getMD5() { return  PSTR("28af3beedcb84986b8e470dc5470507d");};
    #endif

  };

  class QueryCalibrationState {
    public:
    typedef QueryCalibrationStateRequest Request;
    typedef QueryCalibrationStateResponse Response;
  };

}
#endif
