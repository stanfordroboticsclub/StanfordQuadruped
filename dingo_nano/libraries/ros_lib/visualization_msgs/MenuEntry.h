#ifndef _ROS_visualization_msgs_MenuEntry_h
#define _ROS_visualization_msgs_MenuEntry_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace visualization_msgs
{

  class MenuEntry : public ros::Msg
  {
    public:
      typedef uint32_t _id_type;
      _id_type id;
      typedef uint32_t _parent_id_type;
      _parent_id_type parent_id;
      typedef const char* _title_type;
      _title_type title;
      typedef const char* _command_type;
      _command_type command;
      typedef uint8_t _command_type_type;
      _command_type_type command_type;
      enum { FEEDBACK = 0 };
      enum { ROSRUN = 1 };
      enum { ROSLAUNCH = 2 };

    MenuEntry():
      id(0),
      parent_id(0),
      title(""),
      command(""),
      command_type(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      *(outbuffer + offset + 0) = (this->parent_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->parent_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->parent_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->parent_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->parent_id);
      uint32_t length_title = strlen(this->title);
      varToArr(outbuffer + offset, length_title);
      offset += 4;
      memcpy(outbuffer + offset, this->title, length_title);
      offset += length_title;
      uint32_t length_command = strlen(this->command);
      varToArr(outbuffer + offset, length_command);
      offset += 4;
      memcpy(outbuffer + offset, this->command, length_command);
      offset += length_command;
      *(outbuffer + offset + 0) = (this->command_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->command_type);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->id =  ((uint32_t) (*(inbuffer + offset)));
      this->id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->id);
      this->parent_id =  ((uint32_t) (*(inbuffer + offset)));
      this->parent_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->parent_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->parent_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->parent_id);
      uint32_t length_title;
      arrToVar(length_title, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_title; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_title-1]=0;
      this->title = (char *)(inbuffer + offset-1);
      offset += length_title;
      uint32_t length_command;
      arrToVar(length_command, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_command; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_command-1]=0;
      this->command = (char *)(inbuffer + offset-1);
      offset += length_command;
      this->command_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->command_type);
     return offset;
    }

    virtual const char * getType() override { return "visualization_msgs/MenuEntry"; };
    virtual const char * getMD5() override { return "b90ec63024573de83b57aa93eb39be2d"; };

  };

}
#endif
