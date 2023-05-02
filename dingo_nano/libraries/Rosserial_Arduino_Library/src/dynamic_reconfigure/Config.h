#ifndef _ROS_dynamic_reconfigure_Config_h
#define _ROS_dynamic_reconfigure_Config_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "dynamic_reconfigure/BoolParameter.h"
#include "dynamic_reconfigure/IntParameter.h"
#include "dynamic_reconfigure/StrParameter.h"
#include "dynamic_reconfigure/DoubleParameter.h"
#include "dynamic_reconfigure/GroupState.h"

namespace dynamic_reconfigure
{

  class Config : public ros::Msg
  {
    public:
      uint32_t bools_length;
      typedef dynamic_reconfigure::BoolParameter _bools_type;
      _bools_type st_bools;
      _bools_type * bools;
      uint32_t ints_length;
      typedef dynamic_reconfigure::IntParameter _ints_type;
      _ints_type st_ints;
      _ints_type * ints;
      uint32_t strs_length;
      typedef dynamic_reconfigure::StrParameter _strs_type;
      _strs_type st_strs;
      _strs_type * strs;
      uint32_t doubles_length;
      typedef dynamic_reconfigure::DoubleParameter _doubles_type;
      _doubles_type st_doubles;
      _doubles_type * doubles;
      uint32_t groups_length;
      typedef dynamic_reconfigure::GroupState _groups_type;
      _groups_type st_groups;
      _groups_type * groups;

    Config():
      bools_length(0), st_bools(), bools(nullptr),
      ints_length(0), st_ints(), ints(nullptr),
      strs_length(0), st_strs(), strs(nullptr),
      doubles_length(0), st_doubles(), doubles(nullptr),
      groups_length(0), st_groups(), groups(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->bools_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->bools_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->bools_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->bools_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bools_length);
      for( uint32_t i = 0; i < bools_length; i++){
      offset += this->bools[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->ints_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ints_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ints_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ints_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ints_length);
      for( uint32_t i = 0; i < ints_length; i++){
      offset += this->ints[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->strs_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->strs_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->strs_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->strs_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->strs_length);
      for( uint32_t i = 0; i < strs_length; i++){
      offset += this->strs[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->doubles_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->doubles_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->doubles_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->doubles_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->doubles_length);
      for( uint32_t i = 0; i < doubles_length; i++){
      offset += this->doubles[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->groups_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->groups_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->groups_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->groups_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->groups_length);
      for( uint32_t i = 0; i < groups_length; i++){
      offset += this->groups[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t bools_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      bools_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      bools_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      bools_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->bools_length);
      if(bools_lengthT > bools_length)
        this->bools = (dynamic_reconfigure::BoolParameter*)realloc(this->bools, bools_lengthT * sizeof(dynamic_reconfigure::BoolParameter));
      bools_length = bools_lengthT;
      for( uint32_t i = 0; i < bools_length; i++){
      offset += this->st_bools.deserialize(inbuffer + offset);
        memcpy( &(this->bools[i]), &(this->st_bools), sizeof(dynamic_reconfigure::BoolParameter));
      }
      uint32_t ints_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      ints_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      ints_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      ints_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->ints_length);
      if(ints_lengthT > ints_length)
        this->ints = (dynamic_reconfigure::IntParameter*)realloc(this->ints, ints_lengthT * sizeof(dynamic_reconfigure::IntParameter));
      ints_length = ints_lengthT;
      for( uint32_t i = 0; i < ints_length; i++){
      offset += this->st_ints.deserialize(inbuffer + offset);
        memcpy( &(this->ints[i]), &(this->st_ints), sizeof(dynamic_reconfigure::IntParameter));
      }
      uint32_t strs_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      strs_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      strs_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      strs_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->strs_length);
      if(strs_lengthT > strs_length)
        this->strs = (dynamic_reconfigure::StrParameter*)realloc(this->strs, strs_lengthT * sizeof(dynamic_reconfigure::StrParameter));
      strs_length = strs_lengthT;
      for( uint32_t i = 0; i < strs_length; i++){
      offset += this->st_strs.deserialize(inbuffer + offset);
        memcpy( &(this->strs[i]), &(this->st_strs), sizeof(dynamic_reconfigure::StrParameter));
      }
      uint32_t doubles_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      doubles_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      doubles_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      doubles_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->doubles_length);
      if(doubles_lengthT > doubles_length)
        this->doubles = (dynamic_reconfigure::DoubleParameter*)realloc(this->doubles, doubles_lengthT * sizeof(dynamic_reconfigure::DoubleParameter));
      doubles_length = doubles_lengthT;
      for( uint32_t i = 0; i < doubles_length; i++){
      offset += this->st_doubles.deserialize(inbuffer + offset);
        memcpy( &(this->doubles[i]), &(this->st_doubles), sizeof(dynamic_reconfigure::DoubleParameter));
      }
      uint32_t groups_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      groups_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      groups_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      groups_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->groups_length);
      if(groups_lengthT > groups_length)
        this->groups = (dynamic_reconfigure::GroupState*)realloc(this->groups, groups_lengthT * sizeof(dynamic_reconfigure::GroupState));
      groups_length = groups_lengthT;
      for( uint32_t i = 0; i < groups_length; i++){
      offset += this->st_groups.deserialize(inbuffer + offset);
        memcpy( &(this->groups[i]), &(this->st_groups), sizeof(dynamic_reconfigure::GroupState));
      }
     return offset;
    }

    virtual const char * getType() override { return "dynamic_reconfigure/Config"; };
    virtual const char * getMD5() override { return "958f16a05573709014982821e6822580"; };

  };

}
#endif
