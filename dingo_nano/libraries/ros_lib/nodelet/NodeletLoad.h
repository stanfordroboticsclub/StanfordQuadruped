#ifndef _ROS_SERVICE_NodeletLoad_h
#define _ROS_SERVICE_NodeletLoad_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace nodelet
{

static const char NODELETLOAD[] = "nodelet/NodeletLoad";

  class NodeletLoadRequest : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _type_type;
      _type_type type;
      uint32_t remap_source_args_length;
      typedef char* _remap_source_args_type;
      _remap_source_args_type st_remap_source_args;
      _remap_source_args_type * remap_source_args;
      uint32_t remap_target_args_length;
      typedef char* _remap_target_args_type;
      _remap_target_args_type st_remap_target_args;
      _remap_target_args_type * remap_target_args;
      uint32_t my_argv_length;
      typedef char* _my_argv_type;
      _my_argv_type st_my_argv;
      _my_argv_type * my_argv;
      typedef const char* _bond_id_type;
      _bond_id_type bond_id;

    NodeletLoadRequest():
      name(""),
      type(""),
      remap_source_args_length(0), st_remap_source_args(), remap_source_args(nullptr),
      remap_target_args_length(0), st_remap_target_args(), remap_target_args(nullptr),
      my_argv_length(0), st_my_argv(), my_argv(nullptr),
      bond_id("")
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
      *(outbuffer + offset + 0) = (this->remap_source_args_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->remap_source_args_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->remap_source_args_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->remap_source_args_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->remap_source_args_length);
      for( uint32_t i = 0; i < remap_source_args_length; i++){
      uint32_t length_remap_source_argsi = strlen(this->remap_source_args[i]);
      varToArr(outbuffer + offset, length_remap_source_argsi);
      offset += 4;
      memcpy(outbuffer + offset, this->remap_source_args[i], length_remap_source_argsi);
      offset += length_remap_source_argsi;
      }
      *(outbuffer + offset + 0) = (this->remap_target_args_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->remap_target_args_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->remap_target_args_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->remap_target_args_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->remap_target_args_length);
      for( uint32_t i = 0; i < remap_target_args_length; i++){
      uint32_t length_remap_target_argsi = strlen(this->remap_target_args[i]);
      varToArr(outbuffer + offset, length_remap_target_argsi);
      offset += 4;
      memcpy(outbuffer + offset, this->remap_target_args[i], length_remap_target_argsi);
      offset += length_remap_target_argsi;
      }
      *(outbuffer + offset + 0) = (this->my_argv_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->my_argv_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->my_argv_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->my_argv_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->my_argv_length);
      for( uint32_t i = 0; i < my_argv_length; i++){
      uint32_t length_my_argvi = strlen(this->my_argv[i]);
      varToArr(outbuffer + offset, length_my_argvi);
      offset += 4;
      memcpy(outbuffer + offset, this->my_argv[i], length_my_argvi);
      offset += length_my_argvi;
      }
      uint32_t length_bond_id = strlen(this->bond_id);
      varToArr(outbuffer + offset, length_bond_id);
      offset += 4;
      memcpy(outbuffer + offset, this->bond_id, length_bond_id);
      offset += length_bond_id;
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
      uint32_t remap_source_args_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      remap_source_args_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      remap_source_args_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      remap_source_args_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->remap_source_args_length);
      if(remap_source_args_lengthT > remap_source_args_length)
        this->remap_source_args = (char**)realloc(this->remap_source_args, remap_source_args_lengthT * sizeof(char*));
      remap_source_args_length = remap_source_args_lengthT;
      for( uint32_t i = 0; i < remap_source_args_length; i++){
      uint32_t length_st_remap_source_args;
      arrToVar(length_st_remap_source_args, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_remap_source_args; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_remap_source_args-1]=0;
      this->st_remap_source_args = (char *)(inbuffer + offset-1);
      offset += length_st_remap_source_args;
        memcpy( &(this->remap_source_args[i]), &(this->st_remap_source_args), sizeof(char*));
      }
      uint32_t remap_target_args_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      remap_target_args_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      remap_target_args_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      remap_target_args_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->remap_target_args_length);
      if(remap_target_args_lengthT > remap_target_args_length)
        this->remap_target_args = (char**)realloc(this->remap_target_args, remap_target_args_lengthT * sizeof(char*));
      remap_target_args_length = remap_target_args_lengthT;
      for( uint32_t i = 0; i < remap_target_args_length; i++){
      uint32_t length_st_remap_target_args;
      arrToVar(length_st_remap_target_args, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_remap_target_args; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_remap_target_args-1]=0;
      this->st_remap_target_args = (char *)(inbuffer + offset-1);
      offset += length_st_remap_target_args;
        memcpy( &(this->remap_target_args[i]), &(this->st_remap_target_args), sizeof(char*));
      }
      uint32_t my_argv_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      my_argv_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      my_argv_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      my_argv_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->my_argv_length);
      if(my_argv_lengthT > my_argv_length)
        this->my_argv = (char**)realloc(this->my_argv, my_argv_lengthT * sizeof(char*));
      my_argv_length = my_argv_lengthT;
      for( uint32_t i = 0; i < my_argv_length; i++){
      uint32_t length_st_my_argv;
      arrToVar(length_st_my_argv, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_my_argv; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_my_argv-1]=0;
      this->st_my_argv = (char *)(inbuffer + offset-1);
      offset += length_st_my_argv;
        memcpy( &(this->my_argv[i]), &(this->st_my_argv), sizeof(char*));
      }
      uint32_t length_bond_id;
      arrToVar(length_bond_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_bond_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_bond_id-1]=0;
      this->bond_id = (char *)(inbuffer + offset-1);
      offset += length_bond_id;
     return offset;
    }

    virtual const char * getType() override { return NODELETLOAD; };
    virtual const char * getMD5() override { return "c6e28cc4d2e259249d96cfb50658fbec"; };

  };

  class NodeletLoadResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    NodeletLoadResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
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
     return offset;
    }

    virtual const char * getType() override { return NODELETLOAD; };
    virtual const char * getMD5() override { return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class NodeletLoad {
    public:
    typedef NodeletLoadRequest Request;
    typedef NodeletLoadResponse Response;
  };

}
#endif
