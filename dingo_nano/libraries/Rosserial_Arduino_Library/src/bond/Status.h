#ifndef _ROS_bond_Status_h
#define _ROS_bond_Status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace bond
{

  class Status : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _id_type;
      _id_type id;
      typedef const char* _instance_id_type;
      _instance_id_type instance_id;
      typedef bool _active_type;
      _active_type active;
      typedef float _heartbeat_timeout_type;
      _heartbeat_timeout_type heartbeat_timeout;
      typedef float _heartbeat_period_type;
      _heartbeat_period_type heartbeat_period;

    Status():
      header(),
      id(""),
      instance_id(""),
      active(0),
      heartbeat_timeout(0),
      heartbeat_period(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_id = strlen(this->id);
      varToArr(outbuffer + offset, length_id);
      offset += 4;
      memcpy(outbuffer + offset, this->id, length_id);
      offset += length_id;
      uint32_t length_instance_id = strlen(this->instance_id);
      varToArr(outbuffer + offset, length_instance_id);
      offset += 4;
      memcpy(outbuffer + offset, this->instance_id, length_instance_id);
      offset += length_instance_id;
      union {
        bool real;
        uint8_t base;
      } u_active;
      u_active.real = this->active;
      *(outbuffer + offset + 0) = (u_active.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->active);
      union {
        float real;
        uint32_t base;
      } u_heartbeat_timeout;
      u_heartbeat_timeout.real = this->heartbeat_timeout;
      *(outbuffer + offset + 0) = (u_heartbeat_timeout.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_heartbeat_timeout.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_heartbeat_timeout.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_heartbeat_timeout.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->heartbeat_timeout);
      union {
        float real;
        uint32_t base;
      } u_heartbeat_period;
      u_heartbeat_period.real = this->heartbeat_period;
      *(outbuffer + offset + 0) = (u_heartbeat_period.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_heartbeat_period.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_heartbeat_period.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_heartbeat_period.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->heartbeat_period);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_id;
      arrToVar(length_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_id-1]=0;
      this->id = (char *)(inbuffer + offset-1);
      offset += length_id;
      uint32_t length_instance_id;
      arrToVar(length_instance_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_instance_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_instance_id-1]=0;
      this->instance_id = (char *)(inbuffer + offset-1);
      offset += length_instance_id;
      union {
        bool real;
        uint8_t base;
      } u_active;
      u_active.base = 0;
      u_active.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->active = u_active.real;
      offset += sizeof(this->active);
      union {
        float real;
        uint32_t base;
      } u_heartbeat_timeout;
      u_heartbeat_timeout.base = 0;
      u_heartbeat_timeout.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_heartbeat_timeout.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_heartbeat_timeout.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_heartbeat_timeout.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->heartbeat_timeout = u_heartbeat_timeout.real;
      offset += sizeof(this->heartbeat_timeout);
      union {
        float real;
        uint32_t base;
      } u_heartbeat_period;
      u_heartbeat_period.base = 0;
      u_heartbeat_period.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_heartbeat_period.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_heartbeat_period.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_heartbeat_period.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->heartbeat_period = u_heartbeat_period.real;
      offset += sizeof(this->heartbeat_period);
     return offset;
    }

    virtual const char * getType() override { return "bond/Status"; };
    virtual const char * getMD5() override { return "eacc84bf5d65b6777d4c50f463dfb9c8"; };

  };

}
#endif
