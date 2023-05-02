#ifndef _ROS_diagnostic_msgs_DiagnosticStatus_h
#define _ROS_diagnostic_msgs_DiagnosticStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "diagnostic_msgs/KeyValue.h"

namespace diagnostic_msgs
{

  class DiagnosticStatus : public ros::Msg
  {
    public:
      typedef int8_t _level_type;
      _level_type level;
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _message_type;
      _message_type message;
      typedef const char* _hardware_id_type;
      _hardware_id_type hardware_id;
      uint32_t values_length;
      typedef diagnostic_msgs::KeyValue _values_type;
      _values_type st_values;
      _values_type * values;
      enum { OK = 0 };
      enum { WARN = 1 };
      enum { ERROR = 2 };
      enum { STALE = 3 };

    DiagnosticStatus():
      level(0),
      name(""),
      message(""),
      hardware_id(""),
      values_length(0), st_values(), values(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_level;
      u_level.real = this->level;
      *(outbuffer + offset + 0) = (u_level.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->level);
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      uint32_t length_hardware_id = strlen(this->hardware_id);
      varToArr(outbuffer + offset, length_hardware_id);
      offset += 4;
      memcpy(outbuffer + offset, this->hardware_id, length_hardware_id);
      offset += length_hardware_id;
      *(outbuffer + offset + 0) = (this->values_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->values_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->values_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->values_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->values_length);
      for( uint32_t i = 0; i < values_length; i++){
      offset += this->values[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_level;
      u_level.base = 0;
      u_level.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->level = u_level.real;
      offset += sizeof(this->level);
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
      uint32_t length_hardware_id;
      arrToVar(length_hardware_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_hardware_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_hardware_id-1]=0;
      this->hardware_id = (char *)(inbuffer + offset-1);
      offset += length_hardware_id;
      uint32_t values_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      values_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      values_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      values_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->values_length);
      if(values_lengthT > values_length)
        this->values = (diagnostic_msgs::KeyValue*)realloc(this->values, values_lengthT * sizeof(diagnostic_msgs::KeyValue));
      values_length = values_lengthT;
      for( uint32_t i = 0; i < values_length; i++){
      offset += this->st_values.deserialize(inbuffer + offset);
        memcpy( &(this->values[i]), &(this->st_values), sizeof(diagnostic_msgs::KeyValue));
      }
     return offset;
    }

    virtual const char * getType() override { return "diagnostic_msgs/DiagnosticStatus"; };
    virtual const char * getMD5() override { return "d0ce08bc6e5ba34c7754f563a9cabaf1"; };

  };

}
#endif
