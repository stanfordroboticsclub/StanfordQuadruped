#ifndef _ROS_SERVICE_QueryCalibrationState_h
#define _ROS_SERVICE_QueryCalibrationState_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace control_msgs
{

static const char QUERYCALIBRATIONSTATE[] = "control_msgs/QueryCalibrationState";

  class QueryCalibrationStateRequest : public ros::Msg
  {
    public:

    QueryCalibrationStateRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return QUERYCALIBRATIONSTATE; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

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

    virtual int serialize(unsigned char *outbuffer) const override
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

    virtual int deserialize(unsigned char *inbuffer) override
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

    virtual const char * getType() override { return QUERYCALIBRATIONSTATE; };
    virtual const char * getMD5() override { return "28af3beedcb84986b8e470dc5470507d"; };

  };

  class QueryCalibrationState {
    public:
    typedef QueryCalibrationStateRequest Request;
    typedef QueryCalibrationStateResponse Response;
  };

}
#endif
