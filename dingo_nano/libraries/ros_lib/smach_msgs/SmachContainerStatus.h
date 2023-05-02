#ifndef _ROS_smach_msgs_SmachContainerStatus_h
#define _ROS_smach_msgs_SmachContainerStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace smach_msgs
{

  class SmachContainerStatus : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _path_type;
      _path_type path;
      uint32_t initial_states_length;
      typedef char* _initial_states_type;
      _initial_states_type st_initial_states;
      _initial_states_type * initial_states;
      uint32_t active_states_length;
      typedef char* _active_states_type;
      _active_states_type st_active_states;
      _active_states_type * active_states;
      typedef const char* _local_data_type;
      _local_data_type local_data;
      typedef const char* _info_type;
      _info_type info;

    SmachContainerStatus():
      header(),
      path(""),
      initial_states_length(0), st_initial_states(), initial_states(nullptr),
      active_states_length(0), st_active_states(), active_states(nullptr),
      local_data(""),
      info("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_path = strlen(this->path);
      varToArr(outbuffer + offset, length_path);
      offset += 4;
      memcpy(outbuffer + offset, this->path, length_path);
      offset += length_path;
      *(outbuffer + offset + 0) = (this->initial_states_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->initial_states_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->initial_states_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->initial_states_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->initial_states_length);
      for( uint32_t i = 0; i < initial_states_length; i++){
      uint32_t length_initial_statesi = strlen(this->initial_states[i]);
      varToArr(outbuffer + offset, length_initial_statesi);
      offset += 4;
      memcpy(outbuffer + offset, this->initial_states[i], length_initial_statesi);
      offset += length_initial_statesi;
      }
      *(outbuffer + offset + 0) = (this->active_states_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->active_states_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->active_states_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->active_states_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->active_states_length);
      for( uint32_t i = 0; i < active_states_length; i++){
      uint32_t length_active_statesi = strlen(this->active_states[i]);
      varToArr(outbuffer + offset, length_active_statesi);
      offset += 4;
      memcpy(outbuffer + offset, this->active_states[i], length_active_statesi);
      offset += length_active_statesi;
      }
      uint32_t length_local_data = strlen(this->local_data);
      varToArr(outbuffer + offset, length_local_data);
      offset += 4;
      memcpy(outbuffer + offset, this->local_data, length_local_data);
      offset += length_local_data;
      uint32_t length_info = strlen(this->info);
      varToArr(outbuffer + offset, length_info);
      offset += 4;
      memcpy(outbuffer + offset, this->info, length_info);
      offset += length_info;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_path;
      arrToVar(length_path, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_path; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_path-1]=0;
      this->path = (char *)(inbuffer + offset-1);
      offset += length_path;
      uint32_t initial_states_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      initial_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      initial_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      initial_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->initial_states_length);
      if(initial_states_lengthT > initial_states_length)
        this->initial_states = (char**)realloc(this->initial_states, initial_states_lengthT * sizeof(char*));
      initial_states_length = initial_states_lengthT;
      for( uint32_t i = 0; i < initial_states_length; i++){
      uint32_t length_st_initial_states;
      arrToVar(length_st_initial_states, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_initial_states; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_initial_states-1]=0;
      this->st_initial_states = (char *)(inbuffer + offset-1);
      offset += length_st_initial_states;
        memcpy( &(this->initial_states[i]), &(this->st_initial_states), sizeof(char*));
      }
      uint32_t active_states_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      active_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      active_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      active_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->active_states_length);
      if(active_states_lengthT > active_states_length)
        this->active_states = (char**)realloc(this->active_states, active_states_lengthT * sizeof(char*));
      active_states_length = active_states_lengthT;
      for( uint32_t i = 0; i < active_states_length; i++){
      uint32_t length_st_active_states;
      arrToVar(length_st_active_states, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_active_states; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_active_states-1]=0;
      this->st_active_states = (char *)(inbuffer + offset-1);
      offset += length_st_active_states;
        memcpy( &(this->active_states[i]), &(this->st_active_states), sizeof(char*));
      }
      uint32_t length_local_data;
      arrToVar(length_local_data, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_local_data; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_local_data-1]=0;
      this->local_data = (char *)(inbuffer + offset-1);
      offset += length_local_data;
      uint32_t length_info;
      arrToVar(length_info, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_info; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_info-1]=0;
      this->info = (char *)(inbuffer + offset-1);
      offset += length_info;
     return offset;
    }

    virtual const char * getType() override { return "smach_msgs/SmachContainerStatus"; };
    virtual const char * getMD5() override { return "5ba2bb79ac19e3842d562a191f2a675b"; };

  };

}
#endif
