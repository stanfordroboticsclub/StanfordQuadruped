#ifndef _ROS_geometry_msgs_AccelStamped_h
#define _ROS_geometry_msgs_AccelStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Accel.h"

namespace geometry_msgs
{

  class AccelStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Accel _accel_type;
      _accel_type accel;

    AccelStamped():
      header(),
      accel()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->accel.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->accel.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "geometry_msgs/AccelStamped"; };
    virtual const char * getMD5() override { return "d8a98a5d81351b6eb0578c78557e7659"; };

  };

}
#endif
