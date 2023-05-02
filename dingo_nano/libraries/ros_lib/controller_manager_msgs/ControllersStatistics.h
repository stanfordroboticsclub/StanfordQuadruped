#ifndef _ROS_controller_manager_msgs_ControllersStatistics_h
#define _ROS_controller_manager_msgs_ControllersStatistics_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "controller_manager_msgs/ControllerStatistics.h"

namespace controller_manager_msgs
{

  class ControllersStatistics : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t controller_length;
      typedef controller_manager_msgs::ControllerStatistics _controller_type;
      _controller_type st_controller;
      _controller_type * controller;

    ControllersStatistics():
      header(),
      controller_length(0), st_controller(), controller(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->controller_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->controller_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->controller_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->controller_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->controller_length);
      for( uint32_t i = 0; i < controller_length; i++){
      offset += this->controller[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t controller_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      controller_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      controller_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      controller_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->controller_length);
      if(controller_lengthT > controller_length)
        this->controller = (controller_manager_msgs::ControllerStatistics*)realloc(this->controller, controller_lengthT * sizeof(controller_manager_msgs::ControllerStatistics));
      controller_length = controller_lengthT;
      for( uint32_t i = 0; i < controller_length; i++){
      offset += this->st_controller.deserialize(inbuffer + offset);
        memcpy( &(this->controller[i]), &(this->st_controller), sizeof(controller_manager_msgs::ControllerStatistics));
      }
     return offset;
    }

    virtual const char * getType() override { return "controller_manager_msgs/ControllersStatistics"; };
    virtual const char * getMD5() override { return "a154c347736773e3700d1719105df29d"; };

  };

}
#endif
