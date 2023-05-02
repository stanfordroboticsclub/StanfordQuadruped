#ifndef _ROS_smach_msgs_SmachContainerInitialStatusCmd_h
#define _ROS_smach_msgs_SmachContainerInitialStatusCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace smach_msgs
{

  class SmachContainerInitialStatusCmd : public ros::Msg
  {
    public:
      typedef const char* _path_type;
      _path_type path;
      uint32_t initial_states_length;
      typedef char* _initial_states_type;
      _initial_states_type st_initial_states;
      _initial_states_type * initial_states;
      typedef const char* _local_data_type;
      _local_data_type local_data;

    SmachContainerInitialStatusCmd():
      path(""),
      initial_states_length(0), st_initial_states(), initial_states(nullptr),
      local_data("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
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
      uint32_t length_local_data = strlen(this->local_data);
      varToArr(outbuffer + offset, length_local_data);
      offset += 4;
      memcpy(outbuffer + offset, this->local_data, length_local_data);
      offset += length_local_data;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
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
      uint32_t length_local_data;
      arrToVar(length_local_data, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_local_data; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_local_data-1]=0;
      this->local_data = (char *)(inbuffer + offset-1);
      offset += length_local_data;
     return offset;
    }

    virtual const char * getType() override { return "smach_msgs/SmachContainerInitialStatusCmd"; };
    virtual const char * getMD5() override { return "45f8cf31fc29b829db77f23001f788d6"; };

  };

}
#endif
