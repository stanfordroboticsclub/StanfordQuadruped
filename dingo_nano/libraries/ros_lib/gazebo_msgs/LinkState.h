#ifndef _ROS_gazebo_msgs_LinkState_h
#define _ROS_gazebo_msgs_LinkState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

namespace gazebo_msgs
{

  class LinkState : public ros::Msg
  {
    public:
      typedef const char* _link_name_type;
      _link_name_type link_name;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef geometry_msgs::Twist _twist_type;
      _twist_type twist;
      typedef const char* _reference_frame_type;
      _reference_frame_type reference_frame;

    LinkState():
      link_name(""),
      pose(),
      twist(),
      reference_frame("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_link_name = strlen(this->link_name);
      varToArr(outbuffer + offset, length_link_name);
      offset += 4;
      memcpy(outbuffer + offset, this->link_name, length_link_name);
      offset += length_link_name;
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->twist.serialize(outbuffer + offset);
      uint32_t length_reference_frame = strlen(this->reference_frame);
      varToArr(outbuffer + offset, length_reference_frame);
      offset += 4;
      memcpy(outbuffer + offset, this->reference_frame, length_reference_frame);
      offset += length_reference_frame;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_link_name;
      arrToVar(length_link_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_link_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_link_name-1]=0;
      this->link_name = (char *)(inbuffer + offset-1);
      offset += length_link_name;
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->twist.deserialize(inbuffer + offset);
      uint32_t length_reference_frame;
      arrToVar(length_reference_frame, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_reference_frame; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_reference_frame-1]=0;
      this->reference_frame = (char *)(inbuffer + offset-1);
      offset += length_reference_frame;
     return offset;
    }

    virtual const char * getType() override { return "gazebo_msgs/LinkState"; };
    virtual const char * getMD5() override { return "0818ebbf28ce3a08d48ab1eaa7309ebe"; };

  };

}
#endif
