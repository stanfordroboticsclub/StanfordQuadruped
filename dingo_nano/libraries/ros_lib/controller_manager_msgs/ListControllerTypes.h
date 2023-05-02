#ifndef _ROS_SERVICE_ListControllerTypes_h
#define _ROS_SERVICE_ListControllerTypes_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace controller_manager_msgs
{

static const char LISTCONTROLLERTYPES[] = "controller_manager_msgs/ListControllerTypes";

  class ListControllerTypesRequest : public ros::Msg
  {
    public:

    ListControllerTypesRequest()
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

    virtual const char * getType() override { return LISTCONTROLLERTYPES; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ListControllerTypesResponse : public ros::Msg
  {
    public:
      uint32_t types_length;
      typedef char* _types_type;
      _types_type st_types;
      _types_type * types;
      uint32_t base_classes_length;
      typedef char* _base_classes_type;
      _base_classes_type st_base_classes;
      _base_classes_type * base_classes;

    ListControllerTypesResponse():
      types_length(0), st_types(), types(nullptr),
      base_classes_length(0), st_base_classes(), base_classes(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->types_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->types_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->types_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->types_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->types_length);
      for( uint32_t i = 0; i < types_length; i++){
      uint32_t length_typesi = strlen(this->types[i]);
      varToArr(outbuffer + offset, length_typesi);
      offset += 4;
      memcpy(outbuffer + offset, this->types[i], length_typesi);
      offset += length_typesi;
      }
      *(outbuffer + offset + 0) = (this->base_classes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->base_classes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->base_classes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->base_classes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_classes_length);
      for( uint32_t i = 0; i < base_classes_length; i++){
      uint32_t length_base_classesi = strlen(this->base_classes[i]);
      varToArr(outbuffer + offset, length_base_classesi);
      offset += 4;
      memcpy(outbuffer + offset, this->base_classes[i], length_base_classesi);
      offset += length_base_classesi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t types_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      types_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      types_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      types_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->types_length);
      if(types_lengthT > types_length)
        this->types = (char**)realloc(this->types, types_lengthT * sizeof(char*));
      types_length = types_lengthT;
      for( uint32_t i = 0; i < types_length; i++){
      uint32_t length_st_types;
      arrToVar(length_st_types, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_types; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_types-1]=0;
      this->st_types = (char *)(inbuffer + offset-1);
      offset += length_st_types;
        memcpy( &(this->types[i]), &(this->st_types), sizeof(char*));
      }
      uint32_t base_classes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      base_classes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      base_classes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      base_classes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->base_classes_length);
      if(base_classes_lengthT > base_classes_length)
        this->base_classes = (char**)realloc(this->base_classes, base_classes_lengthT * sizeof(char*));
      base_classes_length = base_classes_lengthT;
      for( uint32_t i = 0; i < base_classes_length; i++){
      uint32_t length_st_base_classes;
      arrToVar(length_st_base_classes, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_base_classes; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_base_classes-1]=0;
      this->st_base_classes = (char *)(inbuffer + offset-1);
      offset += length_st_base_classes;
        memcpy( &(this->base_classes[i]), &(this->st_base_classes), sizeof(char*));
      }
     return offset;
    }

    virtual const char * getType() override { return LISTCONTROLLERTYPES; };
    virtual const char * getMD5() override { return "c1d4cd11aefa9f97ba4aeb5b33987f4e"; };

  };

  class ListControllerTypes {
    public:
    typedef ListControllerTypesRequest Request;
    typedef ListControllerTypesResponse Response;
  };

}
#endif
