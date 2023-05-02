#ifndef _ROS_visualization_msgs_InteractiveMarkerFeedback_h
#define _ROS_visualization_msgs_InteractiveMarkerFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

namespace visualization_msgs
{

  class InteractiveMarkerFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _client_id_type;
      _client_id_type client_id;
      typedef const char* _marker_name_type;
      _marker_name_type marker_name;
      typedef const char* _control_name_type;
      _control_name_type control_name;
      typedef uint8_t _event_type_type;
      _event_type_type event_type;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef uint32_t _menu_entry_id_type;
      _menu_entry_id_type menu_entry_id;
      typedef geometry_msgs::Point _mouse_point_type;
      _mouse_point_type mouse_point;
      typedef bool _mouse_point_valid_type;
      _mouse_point_valid_type mouse_point_valid;
      enum { KEEP_ALIVE =  0 };
      enum { POSE_UPDATE =  1 };
      enum { MENU_SELECT =  2 };
      enum { BUTTON_CLICK =  3 };
      enum { MOUSE_DOWN =  4 };
      enum { MOUSE_UP =  5 };

    InteractiveMarkerFeedback():
      header(),
      client_id(""),
      marker_name(""),
      control_name(""),
      event_type(0),
      pose(),
      menu_entry_id(0),
      mouse_point(),
      mouse_point_valid(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_client_id = strlen(this->client_id);
      varToArr(outbuffer + offset, length_client_id);
      offset += 4;
      memcpy(outbuffer + offset, this->client_id, length_client_id);
      offset += length_client_id;
      uint32_t length_marker_name = strlen(this->marker_name);
      varToArr(outbuffer + offset, length_marker_name);
      offset += 4;
      memcpy(outbuffer + offset, this->marker_name, length_marker_name);
      offset += length_marker_name;
      uint32_t length_control_name = strlen(this->control_name);
      varToArr(outbuffer + offset, length_control_name);
      offset += 4;
      memcpy(outbuffer + offset, this->control_name, length_control_name);
      offset += length_control_name;
      *(outbuffer + offset + 0) = (this->event_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->event_type);
      offset += this->pose.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->menu_entry_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->menu_entry_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->menu_entry_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->menu_entry_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->menu_entry_id);
      offset += this->mouse_point.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_mouse_point_valid;
      u_mouse_point_valid.real = this->mouse_point_valid;
      *(outbuffer + offset + 0) = (u_mouse_point_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mouse_point_valid);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_client_id;
      arrToVar(length_client_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_client_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_client_id-1]=0;
      this->client_id = (char *)(inbuffer + offset-1);
      offset += length_client_id;
      uint32_t length_marker_name;
      arrToVar(length_marker_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_marker_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_marker_name-1]=0;
      this->marker_name = (char *)(inbuffer + offset-1);
      offset += length_marker_name;
      uint32_t length_control_name;
      arrToVar(length_control_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_control_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_control_name-1]=0;
      this->control_name = (char *)(inbuffer + offset-1);
      offset += length_control_name;
      this->event_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->event_type);
      offset += this->pose.deserialize(inbuffer + offset);
      this->menu_entry_id =  ((uint32_t) (*(inbuffer + offset)));
      this->menu_entry_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->menu_entry_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->menu_entry_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->menu_entry_id);
      offset += this->mouse_point.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_mouse_point_valid;
      u_mouse_point_valid.base = 0;
      u_mouse_point_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mouse_point_valid = u_mouse_point_valid.real;
      offset += sizeof(this->mouse_point_valid);
     return offset;
    }

    virtual const char * getType() override { return "visualization_msgs/InteractiveMarkerFeedback"; };
    virtual const char * getMD5() override { return "ab0f1eee058667e28c19ff3ffc3f4b78"; };

  };

}
#endif
