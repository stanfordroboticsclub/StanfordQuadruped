#ifndef _ROS_visualization_msgs_InteractiveMarkerUpdate_h
#define _ROS_visualization_msgs_InteractiveMarkerUpdate_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "visualization_msgs/InteractiveMarker.h"
#include "visualization_msgs/InteractiveMarkerPose.h"

namespace visualization_msgs
{

  class InteractiveMarkerUpdate : public ros::Msg
  {
    public:
      typedef const char* _server_id_type;
      _server_id_type server_id;
      typedef uint64_t _seq_num_type;
      _seq_num_type seq_num;
      typedef uint8_t _type_type;
      _type_type type;
      uint32_t markers_length;
      typedef visualization_msgs::InteractiveMarker _markers_type;
      _markers_type st_markers;
      _markers_type * markers;
      uint32_t poses_length;
      typedef visualization_msgs::InteractiveMarkerPose _poses_type;
      _poses_type st_poses;
      _poses_type * poses;
      uint32_t erases_length;
      typedef char* _erases_type;
      _erases_type st_erases;
      _erases_type * erases;
      enum { KEEP_ALIVE =  0 };
      enum { UPDATE =  1 };

    InteractiveMarkerUpdate():
      server_id(""),
      seq_num(0),
      type(0),
      markers_length(0), st_markers(), markers(nullptr),
      poses_length(0), st_poses(), poses(nullptr),
      erases_length(0), st_erases(), erases(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_server_id = strlen(this->server_id);
      varToArr(outbuffer + offset, length_server_id);
      offset += 4;
      memcpy(outbuffer + offset, this->server_id, length_server_id);
      offset += length_server_id;
      *(outbuffer + offset + 0) = (this->seq_num >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->seq_num >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->seq_num >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->seq_num >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->seq_num >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->seq_num >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->seq_num >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->seq_num >> (8 * 7)) & 0xFF;
      offset += sizeof(this->seq_num);
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      *(outbuffer + offset + 0) = (this->markers_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->markers_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->markers_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->markers_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->markers_length);
      for( uint32_t i = 0; i < markers_length; i++){
      offset += this->markers[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->poses_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->poses_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->poses_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->poses_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->poses_length);
      for( uint32_t i = 0; i < poses_length; i++){
      offset += this->poses[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->erases_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->erases_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->erases_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->erases_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->erases_length);
      for( uint32_t i = 0; i < erases_length; i++){
      uint32_t length_erasesi = strlen(this->erases[i]);
      varToArr(outbuffer + offset, length_erasesi);
      offset += 4;
      memcpy(outbuffer + offset, this->erases[i], length_erasesi);
      offset += length_erasesi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_server_id;
      arrToVar(length_server_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_server_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_server_id-1]=0;
      this->server_id = (char *)(inbuffer + offset-1);
      offset += length_server_id;
      this->seq_num =  ((uint64_t) (*(inbuffer + offset)));
      this->seq_num |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->seq_num |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->seq_num |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->seq_num |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->seq_num |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->seq_num |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->seq_num |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->seq_num);
      this->type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->type);
      uint32_t markers_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      markers_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      markers_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      markers_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->markers_length);
      if(markers_lengthT > markers_length)
        this->markers = (visualization_msgs::InteractiveMarker*)realloc(this->markers, markers_lengthT * sizeof(visualization_msgs::InteractiveMarker));
      markers_length = markers_lengthT;
      for( uint32_t i = 0; i < markers_length; i++){
      offset += this->st_markers.deserialize(inbuffer + offset);
        memcpy( &(this->markers[i]), &(this->st_markers), sizeof(visualization_msgs::InteractiveMarker));
      }
      uint32_t poses_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      poses_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      poses_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      poses_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->poses_length);
      if(poses_lengthT > poses_length)
        this->poses = (visualization_msgs::InteractiveMarkerPose*)realloc(this->poses, poses_lengthT * sizeof(visualization_msgs::InteractiveMarkerPose));
      poses_length = poses_lengthT;
      for( uint32_t i = 0; i < poses_length; i++){
      offset += this->st_poses.deserialize(inbuffer + offset);
        memcpy( &(this->poses[i]), &(this->st_poses), sizeof(visualization_msgs::InteractiveMarkerPose));
      }
      uint32_t erases_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      erases_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      erases_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      erases_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->erases_length);
      if(erases_lengthT > erases_length)
        this->erases = (char**)realloc(this->erases, erases_lengthT * sizeof(char*));
      erases_length = erases_lengthT;
      for( uint32_t i = 0; i < erases_length; i++){
      uint32_t length_st_erases;
      arrToVar(length_st_erases, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_erases; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_erases-1]=0;
      this->st_erases = (char *)(inbuffer + offset-1);
      offset += length_st_erases;
        memcpy( &(this->erases[i]), &(this->st_erases), sizeof(char*));
      }
     return offset;
    }

    virtual const char * getType() override { return "visualization_msgs/InteractiveMarkerUpdate"; };
    virtual const char * getMD5() override { return "710d308d0a9276d65945e92dd30b3946"; };

  };

}
#endif
