#ifndef _ROS_smach_msgs_SmachContainerStructure_h
#define _ROS_smach_msgs_SmachContainerStructure_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace smach_msgs
{

  class SmachContainerStructure : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _path_type;
      _path_type path;
      uint32_t children_length;
      typedef char* _children_type;
      _children_type st_children;
      _children_type * children;
      uint32_t internal_outcomes_length;
      typedef char* _internal_outcomes_type;
      _internal_outcomes_type st_internal_outcomes;
      _internal_outcomes_type * internal_outcomes;
      uint32_t outcomes_from_length;
      typedef char* _outcomes_from_type;
      _outcomes_from_type st_outcomes_from;
      _outcomes_from_type * outcomes_from;
      uint32_t outcomes_to_length;
      typedef char* _outcomes_to_type;
      _outcomes_to_type st_outcomes_to;
      _outcomes_to_type * outcomes_to;
      uint32_t container_outcomes_length;
      typedef char* _container_outcomes_type;
      _container_outcomes_type st_container_outcomes;
      _container_outcomes_type * container_outcomes;

    SmachContainerStructure():
      header(),
      path(""),
      children_length(0), st_children(), children(nullptr),
      internal_outcomes_length(0), st_internal_outcomes(), internal_outcomes(nullptr),
      outcomes_from_length(0), st_outcomes_from(), outcomes_from(nullptr),
      outcomes_to_length(0), st_outcomes_to(), outcomes_to(nullptr),
      container_outcomes_length(0), st_container_outcomes(), container_outcomes(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_path = strlen(this->path);
      varToArr(outbuffer + offset, length_path);
      offset += 4;
      memcpy(outbuffer + offset, this->path, length_path);
      offset += length_path;
      *(outbuffer + offset + 0) = (this->children_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->children_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->children_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->children_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->children_length);
      for( uint32_t i = 0; i < children_length; i++){
      uint32_t length_childreni = strlen(this->children[i]);
      varToArr(outbuffer + offset, length_childreni);
      offset += 4;
      memcpy(outbuffer + offset, this->children[i], length_childreni);
      offset += length_childreni;
      }
      *(outbuffer + offset + 0) = (this->internal_outcomes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->internal_outcomes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->internal_outcomes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->internal_outcomes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->internal_outcomes_length);
      for( uint32_t i = 0; i < internal_outcomes_length; i++){
      uint32_t length_internal_outcomesi = strlen(this->internal_outcomes[i]);
      varToArr(outbuffer + offset, length_internal_outcomesi);
      offset += 4;
      memcpy(outbuffer + offset, this->internal_outcomes[i], length_internal_outcomesi);
      offset += length_internal_outcomesi;
      }
      *(outbuffer + offset + 0) = (this->outcomes_from_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->outcomes_from_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->outcomes_from_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->outcomes_from_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->outcomes_from_length);
      for( uint32_t i = 0; i < outcomes_from_length; i++){
      uint32_t length_outcomes_fromi = strlen(this->outcomes_from[i]);
      varToArr(outbuffer + offset, length_outcomes_fromi);
      offset += 4;
      memcpy(outbuffer + offset, this->outcomes_from[i], length_outcomes_fromi);
      offset += length_outcomes_fromi;
      }
      *(outbuffer + offset + 0) = (this->outcomes_to_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->outcomes_to_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->outcomes_to_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->outcomes_to_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->outcomes_to_length);
      for( uint32_t i = 0; i < outcomes_to_length; i++){
      uint32_t length_outcomes_toi = strlen(this->outcomes_to[i]);
      varToArr(outbuffer + offset, length_outcomes_toi);
      offset += 4;
      memcpy(outbuffer + offset, this->outcomes_to[i], length_outcomes_toi);
      offset += length_outcomes_toi;
      }
      *(outbuffer + offset + 0) = (this->container_outcomes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->container_outcomes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->container_outcomes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->container_outcomes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->container_outcomes_length);
      for( uint32_t i = 0; i < container_outcomes_length; i++){
      uint32_t length_container_outcomesi = strlen(this->container_outcomes[i]);
      varToArr(outbuffer + offset, length_container_outcomesi);
      offset += 4;
      memcpy(outbuffer + offset, this->container_outcomes[i], length_container_outcomesi);
      offset += length_container_outcomesi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_path;
      arrToVar(length_path, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_path; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_path-1]=0;
      this->path = (char *)(inbuffer + offset-1);
      offset += length_path;
      uint32_t children_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      children_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      children_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      children_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->children_length);
      if(children_lengthT > children_length)
        this->children = (char**)realloc(this->children, children_lengthT * sizeof(char*));
      children_length = children_lengthT;
      for( uint32_t i = 0; i < children_length; i++){
      uint32_t length_st_children;
      arrToVar(length_st_children, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_children; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_children-1]=0;
      this->st_children = (char *)(inbuffer + offset-1);
      offset += length_st_children;
        memcpy( &(this->children[i]), &(this->st_children), sizeof(char*));
      }
      uint32_t internal_outcomes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      internal_outcomes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      internal_outcomes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      internal_outcomes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->internal_outcomes_length);
      if(internal_outcomes_lengthT > internal_outcomes_length)
        this->internal_outcomes = (char**)realloc(this->internal_outcomes, internal_outcomes_lengthT * sizeof(char*));
      internal_outcomes_length = internal_outcomes_lengthT;
      for( uint32_t i = 0; i < internal_outcomes_length; i++){
      uint32_t length_st_internal_outcomes;
      arrToVar(length_st_internal_outcomes, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_internal_outcomes; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_internal_outcomes-1]=0;
      this->st_internal_outcomes = (char *)(inbuffer + offset-1);
      offset += length_st_internal_outcomes;
        memcpy( &(this->internal_outcomes[i]), &(this->st_internal_outcomes), sizeof(char*));
      }
      uint32_t outcomes_from_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      outcomes_from_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      outcomes_from_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      outcomes_from_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->outcomes_from_length);
      if(outcomes_from_lengthT > outcomes_from_length)
        this->outcomes_from = (char**)realloc(this->outcomes_from, outcomes_from_lengthT * sizeof(char*));
      outcomes_from_length = outcomes_from_lengthT;
      for( uint32_t i = 0; i < outcomes_from_length; i++){
      uint32_t length_st_outcomes_from;
      arrToVar(length_st_outcomes_from, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_outcomes_from; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_outcomes_from-1]=0;
      this->st_outcomes_from = (char *)(inbuffer + offset-1);
      offset += length_st_outcomes_from;
        memcpy( &(this->outcomes_from[i]), &(this->st_outcomes_from), sizeof(char*));
      }
      uint32_t outcomes_to_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      outcomes_to_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      outcomes_to_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      outcomes_to_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->outcomes_to_length);
      if(outcomes_to_lengthT > outcomes_to_length)
        this->outcomes_to = (char**)realloc(this->outcomes_to, outcomes_to_lengthT * sizeof(char*));
      outcomes_to_length = outcomes_to_lengthT;
      for( uint32_t i = 0; i < outcomes_to_length; i++){
      uint32_t length_st_outcomes_to;
      arrToVar(length_st_outcomes_to, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_outcomes_to; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_outcomes_to-1]=0;
      this->st_outcomes_to = (char *)(inbuffer + offset-1);
      offset += length_st_outcomes_to;
        memcpy( &(this->outcomes_to[i]), &(this->st_outcomes_to), sizeof(char*));
      }
      uint32_t container_outcomes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      container_outcomes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      container_outcomes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      container_outcomes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->container_outcomes_length);
      if(container_outcomes_lengthT > container_outcomes_length)
        this->container_outcomes = (char**)realloc(this->container_outcomes, container_outcomes_lengthT * sizeof(char*));
      container_outcomes_length = container_outcomes_lengthT;
      for( uint32_t i = 0; i < container_outcomes_length; i++){
      uint32_t length_st_container_outcomes;
      arrToVar(length_st_container_outcomes, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_container_outcomes; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_container_outcomes-1]=0;
      this->st_container_outcomes = (char *)(inbuffer + offset-1);
      offset += length_st_container_outcomes;
        memcpy( &(this->container_outcomes[i]), &(this->st_container_outcomes), sizeof(char*));
      }
     return offset;
    }

    virtual const char * getType() override { return "smach_msgs/SmachContainerStructure"; };
    virtual const char * getMD5() override { return "3d3d1e0d0f99779ee9e58101a5dcf7ea"; };

  };

}
#endif
