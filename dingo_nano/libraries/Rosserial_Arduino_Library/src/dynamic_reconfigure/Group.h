#ifndef _ROS_dynamic_reconfigure_Group_h
#define _ROS_dynamic_reconfigure_Group_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "dynamic_reconfigure/ParamDescription.h"

namespace dynamic_reconfigure
{

  class Group : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _type_type;
      _type_type type;
      uint32_t parameters_length;
      typedef dynamic_reconfigure::ParamDescription _parameters_type;
      _parameters_type st_parameters;
      _parameters_type * parameters;
      typedef int32_t _parent_type;
      _parent_type parent;
      typedef int32_t _id_type;
      _id_type id;

    Group():
      name(""),
      type(""),
      parameters_length(0), st_parameters(), parameters(nullptr),
      parent(0),
      id(0)
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
      *(outbuffer + offset + 0) = (this->parameters_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->parameters_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->parameters_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->parameters_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->parameters_length);
      for( uint32_t i = 0; i < parameters_length; i++){
      offset += this->parameters[i].serialize(outbuffer + offset);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_parent;
      u_parent.real = this->parent;
      *(outbuffer + offset + 0) = (u_parent.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_parent.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_parent.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_parent.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->parent);
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
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
      uint32_t parameters_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      parameters_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      parameters_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      parameters_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->parameters_length);
      if(parameters_lengthT > parameters_length)
        this->parameters = (dynamic_reconfigure::ParamDescription*)realloc(this->parameters, parameters_lengthT * sizeof(dynamic_reconfigure::ParamDescription));
      parameters_length = parameters_lengthT;
      for( uint32_t i = 0; i < parameters_length; i++){
      offset += this->st_parameters.deserialize(inbuffer + offset);
        memcpy( &(this->parameters[i]), &(this->st_parameters), sizeof(dynamic_reconfigure::ParamDescription));
      }
      union {
        int32_t real;
        uint32_t base;
      } u_parent;
      u_parent.base = 0;
      u_parent.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_parent.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_parent.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_parent.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->parent = u_parent.real;
      offset += sizeof(this->parent);
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->id = u_id.real;
      offset += sizeof(this->id);
     return offset;
    }

    virtual const char * getType() override { return "dynamic_reconfigure/Group"; };
    virtual const char * getMD5() override { return "9e8cd9e9423c94823db3614dd8b1cf7a"; };

  };

}
#endif
