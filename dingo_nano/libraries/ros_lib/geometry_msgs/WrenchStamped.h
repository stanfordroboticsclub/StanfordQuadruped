#ifndef _ROS_geometry_msgs_WrenchStamped_h
#define _ROS_geometry_msgs_WrenchStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Wrench.h"

namespace geometry_msgs
{

  class WrenchStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Wrench _wrench_type;
      _wrench_type wrench;

    WrenchStamped():
      header(),
      wrench()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->wrench.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->wrench.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "geometry_msgs/WrenchStamped"; };
    virtual const char * getMD5() override { return "d78d3cb249ce23087ade7e7d0c40cfa7"; };

  };

}
#endif
