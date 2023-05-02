#ifndef _ROS_SERVICE_GetMap_h
#define _ROS_SERVICE_GetMap_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nav_msgs/OccupancyGrid.h"

namespace nav_msgs
{

static const char GETMAP[] = "nav_msgs/GetMap";

  class GetMapRequest : public ros::Msg
  {
    public:

    GetMapRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return GETMAP; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetMapResponse : public ros::Msg
  {
    public:
      typedef nav_msgs::OccupancyGrid _map_type;
      _map_type map;

    GetMapResponse():
      map()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->map.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->map.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return GETMAP; };
    virtual const char * getMD5() override { return "6cdd0a18e0aff5b0a3ca2326a89b54ff"; };

  };

  class GetMap {
    public:
    typedef GetMapRequest Request;
    typedef GetMapResponse Response;
  };

}
#endif
