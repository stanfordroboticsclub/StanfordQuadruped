#ifndef _ROS_controller_manager_msgs_ControllerState_h
#define _ROS_controller_manager_msgs_ControllerState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "controller_manager_msgs/HardwareInterfaceResources.h"

namespace controller_manager_msgs
{

  class ControllerState : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _state_type;
      _state_type state;
      typedef const char* _type_type;
      _type_type type;
      uint32_t claimed_resources_length;
      typedef controller_manager_msgs::HardwareInterfaceResources _claimed_resources_type;
      _claimed_resources_type st_claimed_resources;
      _claimed_resources_type * claimed_resources;

    ControllerState():
      name(""),
      state(""),
      type(""),
      claimed_resources_length(0), st_claimed_resources(), claimed_resources(nullptr)
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
      uint32_t length_state = strlen(this->state);
      varToArr(outbuffer + offset, length_state);
      offset += 4;
      memcpy(outbuffer + offset, this->state, length_state);
      offset += length_state;
      uint32_t length_type = strlen(this->type);
      varToArr(outbuffer + offset, length_type);
      offset += 4;
      memcpy(outbuffer + offset, this->type, length_type);
      offset += length_type;
      *(outbuffer + offset + 0) = (this->claimed_resources_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->claimed_resources_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->claimed_resources_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->claimed_resources_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->claimed_resources_length);
      for( uint32_t i = 0; i < claimed_resources_length; i++){
      offset += this->claimed_resources[i].serialize(outbuffer + offset);
      }
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
      uint32_t length_state;
      arrToVar(length_state, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_state; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_state-1]=0;
      this->state = (char *)(inbuffer + offset-1);
      offset += length_state;
      uint32_t length_type;
      arrToVar(length_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_type-1]=0;
      this->type = (char *)(inbuffer + offset-1);
      offset += length_type;
      uint32_t claimed_resources_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      claimed_resources_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      claimed_resources_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      claimed_resources_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->claimed_resources_length);
      if(claimed_resources_lengthT > claimed_resources_length)
        this->claimed_resources = (controller_manager_msgs::HardwareInterfaceResources*)realloc(this->claimed_resources, claimed_resources_lengthT * sizeof(controller_manager_msgs::HardwareInterfaceResources));
      claimed_resources_length = claimed_resources_lengthT;
      for( uint32_t i = 0; i < claimed_resources_length; i++){
      offset += this->st_claimed_resources.deserialize(inbuffer + offset);
        memcpy( &(this->claimed_resources[i]), &(this->st_claimed_resources), sizeof(controller_manager_msgs::HardwareInterfaceResources));
      }
     return offset;
    }

    virtual const char * getType() override { return "controller_manager_msgs/ControllerState"; };
    virtual const char * getMD5() override { return "aeb6b261d97793ab74099a3740245272"; };

  };

}
#endif
