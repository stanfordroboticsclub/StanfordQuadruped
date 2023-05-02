#ifndef _ROS_visualization_msgs_InteractiveMarkerControl_h
#define _ROS_visualization_msgs_InteractiveMarkerControl_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Quaternion.h"
#include "visualization_msgs/Marker.h"

namespace visualization_msgs
{

  class InteractiveMarkerControl : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef geometry_msgs::Quaternion _orientation_type;
      _orientation_type orientation;
      typedef uint8_t _orientation_mode_type;
      _orientation_mode_type orientation_mode;
      typedef uint8_t _interaction_mode_type;
      _interaction_mode_type interaction_mode;
      typedef bool _always_visible_type;
      _always_visible_type always_visible;
      uint32_t markers_length;
      typedef visualization_msgs::Marker _markers_type;
      _markers_type st_markers;
      _markers_type * markers;
      typedef bool _independent_marker_orientation_type;
      _independent_marker_orientation_type independent_marker_orientation;
      typedef const char* _description_type;
      _description_type description;
      enum { INHERIT =  0 };
      enum { FIXED =  1 };
      enum { VIEW_FACING =  2 };
      enum { NONE =  0 };
      enum { MENU =  1 };
      enum { BUTTON =  2 };
      enum { MOVE_AXIS =  3 };
      enum { MOVE_PLANE =  4 };
      enum { ROTATE_AXIS =  5 };
      enum { MOVE_ROTATE =  6 };
      enum { MOVE_3D =  7 };
      enum { ROTATE_3D =  8 };
      enum { MOVE_ROTATE_3D =  9 };

    InteractiveMarkerControl():
      name(""),
      orientation(),
      orientation_mode(0),
      interaction_mode(0),
      always_visible(0),
      markers_length(0), st_markers(), markers(nullptr),
      independent_marker_orientation(0),
      description("")
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
      offset += this->orientation.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->orientation_mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->orientation_mode);
      *(outbuffer + offset + 0) = (this->interaction_mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->interaction_mode);
      union {
        bool real;
        uint8_t base;
      } u_always_visible;
      u_always_visible.real = this->always_visible;
      *(outbuffer + offset + 0) = (u_always_visible.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->always_visible);
      *(outbuffer + offset + 0) = (this->markers_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->markers_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->markers_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->markers_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->markers_length);
      for( uint32_t i = 0; i < markers_length; i++){
      offset += this->markers[i].serialize(outbuffer + offset);
      }
      union {
        bool real;
        uint8_t base;
      } u_independent_marker_orientation;
      u_independent_marker_orientation.real = this->independent_marker_orientation;
      *(outbuffer + offset + 0) = (u_independent_marker_orientation.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->independent_marker_orientation);
      uint32_t length_description = strlen(this->description);
      varToArr(outbuffer + offset, length_description);
      offset += 4;
      memcpy(outbuffer + offset, this->description, length_description);
      offset += length_description;
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
      offset += this->orientation.deserialize(inbuffer + offset);
      this->orientation_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->orientation_mode);
      this->interaction_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->interaction_mode);
      union {
        bool real;
        uint8_t base;
      } u_always_visible;
      u_always_visible.base = 0;
      u_always_visible.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->always_visible = u_always_visible.real;
      offset += sizeof(this->always_visible);
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
      union {
        bool real;
        uint8_t base;
      } u_independent_marker_orientation;
      u_independent_marker_orientation.base = 0;
      u_independent_marker_orientation.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->independent_marker_orientation = u_independent_marker_orientation.real;
      offset += sizeof(this->independent_marker_orientation);
      uint32_t length_description;
      arrToVar(length_description, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_description; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_description-1]=0;
      this->description = (char *)(inbuffer + offset-1);
      offset += length_description;
     return offset;
    }

    virtual const char * getType() override { return "visualization_msgs/InteractiveMarkerControl"; };
    virtual const char * getMD5() override { return "b3c81e785788195d1840b86c28da1aac"; };

  };

}
#endif
