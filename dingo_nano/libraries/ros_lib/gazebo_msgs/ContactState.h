#ifndef _ROS_gazebo_msgs_ContactState_h
#define _ROS_gazebo_msgs_ContactState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Vector3.h"

namespace gazebo_msgs
{

  class ContactState : public ros::Msg
  {
    public:
      typedef const char* _info_type;
      _info_type info;
      typedef const char* _collision1_name_type;
      _collision1_name_type collision1_name;
      typedef const char* _collision2_name_type;
      _collision2_name_type collision2_name;
      uint32_t wrenches_length;
      typedef geometry_msgs::Wrench _wrenches_type;
      _wrenches_type st_wrenches;
      _wrenches_type * wrenches;
      typedef geometry_msgs::Wrench _total_wrench_type;
      _total_wrench_type total_wrench;
      uint32_t contact_positions_length;
      typedef geometry_msgs::Vector3 _contact_positions_type;
      _contact_positions_type st_contact_positions;
      _contact_positions_type * contact_positions;
      uint32_t contact_normals_length;
      typedef geometry_msgs::Vector3 _contact_normals_type;
      _contact_normals_type st_contact_normals;
      _contact_normals_type * contact_normals;
      uint32_t depths_length;
      typedef float _depths_type;
      _depths_type st_depths;
      _depths_type * depths;

    ContactState():
      info(""),
      collision1_name(""),
      collision2_name(""),
      wrenches_length(0), st_wrenches(), wrenches(nullptr),
      total_wrench(),
      contact_positions_length(0), st_contact_positions(), contact_positions(nullptr),
      contact_normals_length(0), st_contact_normals(), contact_normals(nullptr),
      depths_length(0), st_depths(), depths(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_info = strlen(this->info);
      varToArr(outbuffer + offset, length_info);
      offset += 4;
      memcpy(outbuffer + offset, this->info, length_info);
      offset += length_info;
      uint32_t length_collision1_name = strlen(this->collision1_name);
      varToArr(outbuffer + offset, length_collision1_name);
      offset += 4;
      memcpy(outbuffer + offset, this->collision1_name, length_collision1_name);
      offset += length_collision1_name;
      uint32_t length_collision2_name = strlen(this->collision2_name);
      varToArr(outbuffer + offset, length_collision2_name);
      offset += 4;
      memcpy(outbuffer + offset, this->collision2_name, length_collision2_name);
      offset += length_collision2_name;
      *(outbuffer + offset + 0) = (this->wrenches_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->wrenches_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->wrenches_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->wrenches_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wrenches_length);
      for( uint32_t i = 0; i < wrenches_length; i++){
      offset += this->wrenches[i].serialize(outbuffer + offset);
      }
      offset += this->total_wrench.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->contact_positions_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->contact_positions_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->contact_positions_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->contact_positions_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->contact_positions_length);
      for( uint32_t i = 0; i < contact_positions_length; i++){
      offset += this->contact_positions[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->contact_normals_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->contact_normals_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->contact_normals_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->contact_normals_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->contact_normals_length);
      for( uint32_t i = 0; i < contact_normals_length; i++){
      offset += this->contact_normals[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->depths_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->depths_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->depths_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->depths_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->depths_length);
      for( uint32_t i = 0; i < depths_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->depths[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_info;
      arrToVar(length_info, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_info; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_info-1]=0;
      this->info = (char *)(inbuffer + offset-1);
      offset += length_info;
      uint32_t length_collision1_name;
      arrToVar(length_collision1_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_collision1_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_collision1_name-1]=0;
      this->collision1_name = (char *)(inbuffer + offset-1);
      offset += length_collision1_name;
      uint32_t length_collision2_name;
      arrToVar(length_collision2_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_collision2_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_collision2_name-1]=0;
      this->collision2_name = (char *)(inbuffer + offset-1);
      offset += length_collision2_name;
      uint32_t wrenches_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      wrenches_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      wrenches_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      wrenches_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->wrenches_length);
      if(wrenches_lengthT > wrenches_length)
        this->wrenches = (geometry_msgs::Wrench*)realloc(this->wrenches, wrenches_lengthT * sizeof(geometry_msgs::Wrench));
      wrenches_length = wrenches_lengthT;
      for( uint32_t i = 0; i < wrenches_length; i++){
      offset += this->st_wrenches.deserialize(inbuffer + offset);
        memcpy( &(this->wrenches[i]), &(this->st_wrenches), sizeof(geometry_msgs::Wrench));
      }
      offset += this->total_wrench.deserialize(inbuffer + offset);
      uint32_t contact_positions_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      contact_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      contact_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      contact_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->contact_positions_length);
      if(contact_positions_lengthT > contact_positions_length)
        this->contact_positions = (geometry_msgs::Vector3*)realloc(this->contact_positions, contact_positions_lengthT * sizeof(geometry_msgs::Vector3));
      contact_positions_length = contact_positions_lengthT;
      for( uint32_t i = 0; i < contact_positions_length; i++){
      offset += this->st_contact_positions.deserialize(inbuffer + offset);
        memcpy( &(this->contact_positions[i]), &(this->st_contact_positions), sizeof(geometry_msgs::Vector3));
      }
      uint32_t contact_normals_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      contact_normals_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      contact_normals_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      contact_normals_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->contact_normals_length);
      if(contact_normals_lengthT > contact_normals_length)
        this->contact_normals = (geometry_msgs::Vector3*)realloc(this->contact_normals, contact_normals_lengthT * sizeof(geometry_msgs::Vector3));
      contact_normals_length = contact_normals_lengthT;
      for( uint32_t i = 0; i < contact_normals_length; i++){
      offset += this->st_contact_normals.deserialize(inbuffer + offset);
        memcpy( &(this->contact_normals[i]), &(this->st_contact_normals), sizeof(geometry_msgs::Vector3));
      }
      uint32_t depths_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      depths_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      depths_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      depths_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->depths_length);
      if(depths_lengthT > depths_length)
        this->depths = (float*)realloc(this->depths, depths_lengthT * sizeof(float));
      depths_length = depths_lengthT;
      for( uint32_t i = 0; i < depths_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_depths));
        memcpy( &(this->depths[i]), &(this->st_depths), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "gazebo_msgs/ContactState"; };
    virtual const char * getMD5() override { return "48c0ffb054b8c444f870cecea1ee50d9"; };

  };

}
#endif
