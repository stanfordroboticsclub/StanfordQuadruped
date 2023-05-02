#ifndef _ROS_geometry_msgs_TwistWithCovarianceStamped_h
#define _ROS_geometry_msgs_TwistWithCovarianceStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/TwistWithCovariance.h"

namespace geometry_msgs
{

  class TwistWithCovarianceStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::TwistWithCovariance _twist_type;
      _twist_type twist;

    TwistWithCovarianceStamped():
      header(),
      twist()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->twist.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->twist.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "geometry_msgs/TwistWithCovarianceStamped"; };
    virtual const char * getMD5() override { return "8927a1a12fb2607ceea095b2dc440a96"; };

  };

}
#endif
