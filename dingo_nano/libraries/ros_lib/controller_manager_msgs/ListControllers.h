#ifndef _ROS_SERVICE_ListControllers_h
#define _ROS_SERVICE_ListControllers_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "controller_manager_msgs/ControllerState.h"

namespace controller_manager_msgs
{

static const char LISTCONTROLLERS[] = "controller_manager_msgs/ListControllers";

  class ListControllersRequest : public ros::Msg
  {
    public:

    ListControllersRequest()
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

    virtual const char * getType() override { return LISTCONTROLLERS; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ListControllersResponse : public ros::Msg
  {
    public:
      uint32_t controller_length;
      typedef controller_manager_msgs::ControllerState _controller_type;
      _controller_type st_controller;
      _controller_type * controller;

    ListControllersResponse():
      controller_length(0), st_controller(), controller(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
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
      uint32_t controller_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      controller_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      controller_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      controller_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->controller_length);
      if(controller_lengthT > controller_length)
        this->controller = (controller_manager_msgs::ControllerState*)realloc(this->controller, controller_lengthT * sizeof(controller_manager_msgs::ControllerState));
      controller_length = controller_lengthT;
      for( uint32_t i = 0; i < controller_length; i++){
      offset += this->st_controller.deserialize(inbuffer + offset);
        memcpy( &(this->controller[i]), &(this->st_controller), sizeof(controller_manager_msgs::ControllerState));
      }
     return offset;
    }

    virtual const char * getType() override { return LISTCONTROLLERS; };
    virtual const char * getMD5() override { return "1341feb2e63fa791f855565d0da950d8"; };

  };

  class ListControllers {
    public:
    typedef ListControllersRequest Request;
    typedef ListControllersResponse Response;
  };

}
#endif
