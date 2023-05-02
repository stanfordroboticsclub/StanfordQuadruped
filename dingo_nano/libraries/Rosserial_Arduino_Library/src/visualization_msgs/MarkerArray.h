#ifndef _ROS_visualization_msgs_MarkerArray_h
#define _ROS_visualization_msgs_MarkerArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "visualization_msgs/Marker.h"

namespace visualization_msgs
{

  class MarkerArray : public ros::Msg
  {
    public:
      uint32_t markers_length;
      typedef visualization_msgs::Marker _markers_type;
      _markers_type st_markers;
      _markers_type * markers;

    MarkerArray():
      markers_length(0), st_markers(), markers(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->markers_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->markers_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->markers_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->markers_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->markers_length);
      for( uint32_t i = 0; i < markers_length; i++){
      offset += this->markers[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t markers_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      markers_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      markers_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      markers_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->markers_length);
      if(markers_lengthT > markers_length)
        this->markers = (visualization_msgs::Marker*)realloc(this->markers, markers_lengthT * sizeof(visualization_msgs::Marker));
      markers_length = markers_lengthT;
      for( uint32_t i = 0; i < markers_length; i++){
      offset += this->st_markers.deserialize(inbuffer + offset);
        memcpy( &(this->markers[i]), &(this->st_markers), sizeof(visualization_msgs::Marker));
      }
     return offset;
    }

    virtual const char * getType() override { return "visualization_msgs/MarkerArray"; };
    virtual const char * getMD5() override { return "d155b9ce5188fbaf89745847fd5882d7"; };

  };

}
#endif
