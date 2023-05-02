#ifndef _ROS_map_msgs_PointCloud2Update_h
#define _ROS_map_msgs_PointCloud2Update_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/PointCloud2.h"

namespace map_msgs
{

  class PointCloud2Update : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _type_type;
      _type_type type;
      typedef sensor_msgs::PointCloud2 _points_type;
      _points_type points;
      enum { ADD = 0 };
      enum { DELETE = 1 };

    PointCloud2Update():
      header(),
      type(0),
      points()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->type >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->type >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->type >> (8 * 3)) & 0xFF;
      offset += sizeof(this->type);
      offset += this->points.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->type =  ((uint32_t) (*(inbuffer + offset)));
      this->type |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->type |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->type |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->type);
      offset += this->points.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "map_msgs/PointCloud2Update"; };
    virtual const char * getMD5() override { return "6c58e4f249ae9cd2b24fb1ee0f99195e"; };

  };

}
#endif
