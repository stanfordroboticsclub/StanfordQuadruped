#ifndef _ROS_dynamic_reconfigure_ParamDescription_h
#define _ROS_dynamic_reconfigure_ParamDescription_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamic_reconfigure
{

  class ParamDescription : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _type_type;
      _type_type type;
      typedef uint32_t _level_type;
      _level_type level;
      typedef const char* _description_type;
      _description_type description;
      typedef const char* _edit_method_type;
      _edit_method_type edit_method;

    ParamDescription():
      name(""),
      type(""),
      level(0),
      description(""),
      edit_method("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_type = strlen(this->type);
      varToArr(outbuffer + offset, length_type);
      offset += 4;
      memcpy(outbuffer + offset, this->type, length_type);
      offset += length_type;
      *(outbuffer + offset + 0) = (this->level >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->level >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->level >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->level >> (8 * 3)) & 0xFF;
      offset += sizeof(this->level);
      uint32_t length_description = strlen(this->description);
      varToArr(outbuffer + offset, length_description);
      offset += 4;
      memcpy(outbuffer + offset, this->description, length_description);
      offset += length_description;
      uint32_t length_edit_method = strlen(this->edit_method);
      varToArr(outbuffer + offset, length_edit_method);
      offset += 4;
      memcpy(outbuffer + offset, this->edit_method, length_edit_method);
      offset += length_edit_method;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_type;
      arrToVar(length_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_type-1]=0;
      this->type = (char *)(inbuffer + offset-1);
      offset += length_type;
      this->level =  ((uint32_t) (*(inbuffer + offset)));
      this->level |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->level |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->level |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->level);
      uint32_t length_description;
      arrToVar(length_description, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_description; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_description-1]=0;
      this->description = (char *)(inbuffer + offset-1);
      offset += length_description;
      uint32_t length_edit_method;
      arrToVar(length_edit_method, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_edit_method; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_edit_method-1]=0;
      this->edit_method = (char *)(inbuffer + offset-1);
      offset += length_edit_method;
     return offset;
    }

    virtual const char * getType() override { return "dynamic_reconfigure/ParamDescription"; };
    virtual const char * getMD5() override { return "7434fcb9348c13054e0c3b267c8cb34d"; };

  };

}
#endif
