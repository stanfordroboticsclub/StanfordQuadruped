#ifndef _ROS_control_msgs_JointJog_h
#define _ROS_control_msgs_JointJog_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace control_msgs
{

  class JointJog : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t joint_names_length;
      typedef char* _joint_names_type;
      _joint_names_type st_joint_names;
      _joint_names_type * joint_names;
      uint32_t displacements_length;
      typedef float _displacements_type;
      _displacements_type st_displacements;
      _displacements_type * displacements;
      uint32_t velocities_length;
      typedef float _velocities_type;
      _velocities_type st_velocities;
      _velocities_type * velocities;
      typedef float _duration_type;
      _duration_type duration;

    JointJog():
      header(),
      joint_names_length(0), st_joint_names(), joint_names(nullptr),
      displacements_length(0), st_displacements(), displacements(nullptr),
      velocities_length(0), st_velocities(), velocities(nullptr),
      duration(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->joint_names_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_names_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_names_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_names_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_names_length);
      for( uint32_t i = 0; i < joint_names_length; i++){
      uint32_t length_joint_namesi = strlen(this->joint_names[i]);
      varToArr(outbuffer + offset, length_joint_namesi);
      offset += 4;
      memcpy(outbuffer + offset, this->joint_names[i], length_joint_namesi);
      offset += length_joint_namesi;
      }
      *(outbuffer + offset + 0) = (this->displacements_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->displacements_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->displacements_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->displacements_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->displacements_length);
      for( uint32_t i = 0; i < displacements_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->displacements[i]);
      }
      *(outbuffer + offset + 0) = (this->velocities_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->velocities_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->velocities_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->velocities_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocities_length);
      for( uint32_t i = 0; i < velocities_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->velocities[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->duration);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t joint_names_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_names_length);
      if(joint_names_lengthT > joint_names_length)
        this->joint_names = (char**)realloc(this->joint_names, joint_names_lengthT * sizeof(char*));
      joint_names_length = joint_names_lengthT;
      for( uint32_t i = 0; i < joint_names_length; i++){
      uint32_t length_st_joint_names;
      arrToVar(length_st_joint_names, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_joint_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_joint_names-1]=0;
      this->st_joint_names = (char *)(inbuffer + offset-1);
      offset += length_st_joint_names;
        memcpy( &(this->joint_names[i]), &(this->st_joint_names), sizeof(char*));
      }
      uint32_t displacements_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      displacements_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      displacements_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      displacements_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->displacements_length);
      if(displacements_lengthT > displacements_length)
        this->displacements = (float*)realloc(this->displacements, displacements_lengthT * sizeof(float));
      displacements_length = displacements_lengthT;
      for( uint32_t i = 0; i < displacements_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_displacements));
        memcpy( &(this->displacements[i]), &(this->st_displacements), sizeof(float));
      }
      uint32_t velocities_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      velocities_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      velocities_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      velocities_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->velocities_length);
      if(velocities_lengthT > velocities_length)
        this->velocities = (float*)realloc(this->velocities, velocities_lengthT * sizeof(float));
      velocities_length = velocities_lengthT;
      for( uint32_t i = 0; i < velocities_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_velocities));
        memcpy( &(this->velocities[i]), &(this->st_velocities), sizeof(float));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->duration));
     return offset;
    }

    virtual const char * getType() override { return "control_msgs/JointJog"; };
    virtual const char * getMD5() override { return "1685da700c8c2e1254afc92a5fb89c96"; };

  };

}
#endif
