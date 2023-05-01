#ifndef _ROS_dynamic_reconfigure_GroupState_h
#define _ROS_dynamic_reconfigure_GroupState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamic_reconfigure
{

  class GroupState : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef bool _state_type;
      _state_type state;
      typedef int32_t _id_type;
      _id_type id;
      typedef int32_t _parent_type;
      _parent_type parent;

    GroupState():
      name(""),
      state(0),
      id(0),
      parent(0)
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
      union {
        bool real;
        uint8_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
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
      union {
        bool real;
        uint8_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->state = u_state.real;
      offset += sizeof(this->state);
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
     return offset;
    }

    virtual const char * getType() override { return "dynamic_reconfigure/GroupState"; };
    virtual const char * getMD5() override { return "a2d87f51dc22930325041a2f8b1571f8"; };

  };

}
#endif
