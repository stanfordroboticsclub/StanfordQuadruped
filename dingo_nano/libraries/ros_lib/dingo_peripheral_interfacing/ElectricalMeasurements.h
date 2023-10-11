#ifndef _ROS_dingo_peripheral_interfacing_ElectricalMeasurements_h
#define _ROS_dingo_peripheral_interfacing_ElectricalMeasurements_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dingo_peripheral_interfacing
{

  class ElectricalMeasurements : public ros::Msg
  {
    public:
      typedef float _battery_voltage_level_type;
      _battery_voltage_level_type battery_voltage_level;
      typedef float _servo_buck_voltage_level_type;
      _servo_buck_voltage_level_type servo_buck_voltage_level;

    ElectricalMeasurements():
      battery_voltage_level(0),
      servo_buck_voltage_level(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_battery_voltage_level;
      u_battery_voltage_level.real = this->battery_voltage_level;
      *(outbuffer + offset + 0) = (u_battery_voltage_level.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_voltage_level.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_voltage_level.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_voltage_level.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_voltage_level);
      union {
        float real;
        uint32_t base;
      } u_servo_buck_voltage_level;
      u_servo_buck_voltage_level.real = this->servo_buck_voltage_level;
      *(outbuffer + offset + 0) = (u_servo_buck_voltage_level.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo_buck_voltage_level.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo_buck_voltage_level.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo_buck_voltage_level.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->servo_buck_voltage_level);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_battery_voltage_level;
      u_battery_voltage_level.base = 0;
      u_battery_voltage_level.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_voltage_level.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_voltage_level.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_voltage_level.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_voltage_level = u_battery_voltage_level.real;
      offset += sizeof(this->battery_voltage_level);
      union {
        float real;
        uint32_t base;
      } u_servo_buck_voltage_level;
      u_servo_buck_voltage_level.base = 0;
      u_servo_buck_voltage_level.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo_buck_voltage_level.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo_buck_voltage_level.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo_buck_voltage_level.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->servo_buck_voltage_level = u_servo_buck_voltage_level.real;
      offset += sizeof(this->servo_buck_voltage_level);
     return offset;
    }

    virtual const char * getType() override { return "dingo_peripheral_interfacing/ElectricalMeasurements"; };
    virtual const char * getMD5() override { return "7cd8bf648ee5631ca57dfdbcfb5a9043"; };

  };

}
#endif