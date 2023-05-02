#ifndef _ROS_controller_manager_msgs_HardwareInterfaceResources_h
#define _ROS_controller_manager_msgs_HardwareInterfaceResources_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace controller_manager_msgs
{

  class HardwareInterfaceResources : public ros::Msg
  {
    public:
      typedef const char* _hardware_interface_type;
      _hardware_interface_type hardware_interface;
      uint32_t resources_length;
      typedef char* _resources_type;
      _resources_type st_resources;
      _resources_type * resources;

    HardwareInterfaceResources():
      hardware_interface(""),
      resources_length(0), st_resources(), resources(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_hardware_interface = strlen(this->hardware_interface);
      varToArr(outbuffer + offset, length_hardware_interface);
      offset += 4;
      memcpy(outbuffer + offset, this->hardware_interface, length_hardware_interface);
      offset += length_hardware_interface;
      *(outbuffer + offset + 0) = (this->resources_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->resources_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->resources_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->resources_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->resources_length);
      for( uint32_t i = 0; i < resources_length; i++){
      uint32_t length_resourcesi = strlen(this->resources[i]);
      varToArr(outbuffer + offset, length_resourcesi);
      offset += 4;
      memcpy(outbuffer + offset, this->resources[i], length_resourcesi);
      offset += length_resourcesi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_hardware_interface;
      arrToVar(length_hardware_interface, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_hardware_interface; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_hardware_interface-1]=0;
      this->hardware_interface = (char *)(inbuffer + offset-1);
      offset += length_hardware_interface;
      uint32_t resources_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      resources_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      resources_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      resources_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->resources_length);
      if(resources_lengthT > resources_length)
        this->resources = (char**)realloc(this->resources, resources_lengthT * sizeof(char*));
      resources_length = resources_lengthT;
      for( uint32_t i = 0; i < resources_length; i++){
      uint32_t length_st_resources;
      arrToVar(length_st_resources, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_resources; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_resources-1]=0;
      this->st_resources = (char *)(inbuffer + offset-1);
      offset += length_st_resources;
        memcpy( &(this->resources[i]), &(this->st_resources), sizeof(char*));
      }
     return offset;
    }

    virtual const char * getType() override { return "controller_manager_msgs/HardwareInterfaceResources"; };
    virtual const char * getMD5() override { return "f25b55cbf1d1f76e82e5ec9e83f76258"; };

  };

}
#endif
