#ifndef _ROS_SERVICE_DeleteModel_h
#define _ROS_SERVICE_DeleteModel_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"

namespace gazebo_msgs
{

#ifdef ESP8266
    static const char DELETEMODEL[] = "gazebo_msgs/DeleteModel";
#else
    static const char DELETEMODEL[] PROGMEM = "gazebo_msgs/DeleteModel";
#endif

  class DeleteModelRequest : public ros::Msg
  {
    public:
      typedef const char* _model_name_type;
      _model_name_type model_name;

    DeleteModelRequest():
      model_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_model_name = strlen(this->model_name);
      varToArr(outbuffer + offset, length_model_name);
      offset += 4;
      memcpy(outbuffer + offset, this->model_name, length_model_name);
      offset += length_model_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_model_name;
      arrToVar(length_model_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_model_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_model_name-1]=0;
      this->model_name = (char *)(inbuffer + offset-1);
      offset += length_model_name;
     return offset;
    }

    const char * getType(){ return DELETEMODEL; };
    #ifdef ESP8266
        const char * getMD5() { return  ("ea31c8eab6fc401383cf528a7c0984ba");};
    #else
        const char * getMD5() { return  PSTR("ea31c8eab6fc401383cf528a7c0984ba");};
    #endif

  };

  class DeleteModelResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _status_message_type;
      _status_message_type status_message;

    DeleteModelResponse():
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

    const char * getType(){ return DELETEMODEL; };
    #ifdef ESP8266
        const char * getMD5() { return  ("2ec6f3eff0161f4257b808b12bc830c2");};
    #else
        const char * getMD5() { return  PSTR("2ec6f3eff0161f4257b808b12bc830c2");};
    #endif

  };

  class DeleteModel {
    public:
    typedef DeleteModelRequest Request;
    typedef DeleteModelResponse Response;
  };

}
#endif
