#ifndef _ROS_SERVICE_GetWorldProperties_h
#define _ROS_SERVICE_GetWorldProperties_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gazebo_msgs
{

static const char GETWORLDPROPERTIES[] = "gazebo_msgs/GetWorldProperties";

  class GetWorldPropertiesRequest : public ros::Msg
  {
    public:

    GetWorldPropertiesRequest()
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

    virtual const char * getType() override { return GETWORLDPROPERTIES; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetWorldPropertiesResponse : public ros::Msg
  {
    public:
      typedef float _sim_time_type;
      _sim_time_type sim_time;
      uint32_t model_names_length;
      typedef char* _model_names_type;
      _model_names_type st_model_names;
      _model_names_type * model_names;
      typedef bool _rendering_enabled_type;
      _rendering_enabled_type rendering_enabled;
      typedef bool _success_type;
      _success_type success;
      typedef const char* _status_message_type;
      _status_message_type status_message;

    GetWorldPropertiesResponse():
      sim_time(0),
      model_names_length(0), st_model_names(), model_names(nullptr),
      rendering_enabled(0),
      success(0),
      status_message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->sim_time);
      *(outbuffer + offset + 0) = (this->model_names_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->model_names_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->model_names_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->model_names_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->model_names_length);
      for( uint32_t i = 0; i < model_names_length; i++){
      uint32_t length_model_namesi = strlen(this->model_names[i]);
      varToArr(outbuffer + offset, length_model_namesi);
      offset += 4;
      memcpy(outbuffer + offset, this->model_names[i], length_model_namesi);
      offset += length_model_namesi;
      }
      union {
        bool real;
        uint8_t base;
      } u_rendering_enabled;
      u_rendering_enabled.real = this->rendering_enabled;
      *(outbuffer + offset + 0) = (u_rendering_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rendering_enabled);
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

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->sim_time));
      uint32_t model_names_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      model_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      model_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      model_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->model_names_length);
      if(model_names_lengthT > model_names_length)
        this->model_names = (char**)realloc(this->model_names, model_names_lengthT * sizeof(char*));
      model_names_length = model_names_lengthT;
      for( uint32_t i = 0; i < model_names_length; i++){
      uint32_t length_st_model_names;
      arrToVar(length_st_model_names, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_model_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_model_names-1]=0;
      this->st_model_names = (char *)(inbuffer + offset-1);
      offset += length_st_model_names;
        memcpy( &(this->model_names[i]), &(this->st_model_names), sizeof(char*));
      }
      union {
        bool real;
        uint8_t base;
      } u_rendering_enabled;
      u_rendering_enabled.base = 0;
      u_rendering_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->rendering_enabled = u_rendering_enabled.real;
      offset += sizeof(this->rendering_enabled);
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

    virtual const char * getType() override { return GETWORLDPROPERTIES; };
    virtual const char * getMD5() override { return "36bb0f2eccf4d8be971410c22818ba3f"; };

  };

  class GetWorldProperties {
    public:
    typedef GetWorldPropertiesRequest Request;
    typedef GetWorldPropertiesResponse Response;
  };

}
#endif
