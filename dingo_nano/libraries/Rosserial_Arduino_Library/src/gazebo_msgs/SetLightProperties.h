#ifndef _ROS_SERVICE_SetLightProperties_h
#define _ROS_SERVICE_SetLightProperties_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"
#include "std_msgs/ColorRGBA.h"

namespace gazebo_msgs
{

#ifdef ESP8266
    static const char SETLIGHTPROPERTIES[] = "gazebo_msgs/SetLightProperties";
#else
    static const char SETLIGHTPROPERTIES[] PROGMEM = "gazebo_msgs/SetLightProperties";
#endif

  class SetLightPropertiesRequest : public ros::Msg
  {
    public:
      typedef const char* _light_name_type;
      _light_name_type light_name;
      typedef std_msgs::ColorRGBA _diffuse_type;
      _diffuse_type diffuse;
      typedef float _attenuation_constant_type;
      _attenuation_constant_type attenuation_constant;
      typedef float _attenuation_linear_type;
      _attenuation_linear_type attenuation_linear;
      typedef float _attenuation_quadratic_type;
      _attenuation_quadratic_type attenuation_quadratic;

    SetLightPropertiesRequest():
      light_name(""),
      diffuse(),
      attenuation_constant(0),
      attenuation_linear(0),
      attenuation_quadratic(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_light_name = strlen(this->light_name);
      varToArr(outbuffer + offset, length_light_name);
      offset += 4;
      memcpy(outbuffer + offset, this->light_name, length_light_name);
      offset += length_light_name;
      offset += this->diffuse.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->attenuation_constant);
      offset += serializeAvrFloat64(outbuffer + offset, this->attenuation_linear);
      offset += serializeAvrFloat64(outbuffer + offset, this->attenuation_quadratic);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_light_name;
      arrToVar(length_light_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_light_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_light_name-1]=0;
      this->light_name = (char *)(inbuffer + offset-1);
      offset += length_light_name;
      offset += this->diffuse.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->attenuation_constant));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->attenuation_linear));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->attenuation_quadratic));
     return offset;
    }

    const char * getType(){ return SETLIGHTPROPERTIES; };
    #ifdef ESP8266
        const char * getMD5() { return  ("73ad1ac5e9e312ddf7c74f38ad843f34");};
    #else
        const char * getMD5() { return  PSTR("73ad1ac5e9e312ddf7c74f38ad843f34");};
    #endif

  };

  class SetLightPropertiesResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _status_message_type;
      _status_message_type status_message;

    SetLightPropertiesResponse():
      success(0),
      status_message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_status_message = strlen(this->status_message);
      varToArr(outbuffer + offset, length_status_message);
      offset += 4;
      memcpy(outbuffer + offset, this->status_message, length_status_message);
      offset += length_status_message;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_status_message;
      arrToVar(length_status_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_status_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_status_message-1]=0;
      this->status_message = (char *)(inbuffer + offset-1);
      offset += length_status_message;
     return offset;
    }

    const char * getType(){ return SETLIGHTPROPERTIES; };
    #ifdef ESP8266
        const char * getMD5() { return  ("2ec6f3eff0161f4257b808b12bc830c2");};
    #else
        const char * getMD5() { return  PSTR("2ec6f3eff0161f4257b808b12bc830c2");};
    #endif

  };

  class SetLightProperties {
    public:
    typedef SetLightPropertiesRequest Request;
    typedef SetLightPropertiesResponse Response;
  };

}
#endif
