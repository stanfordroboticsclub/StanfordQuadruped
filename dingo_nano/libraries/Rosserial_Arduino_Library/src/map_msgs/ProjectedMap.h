#ifndef _ROS_map_msgs_ProjectedMap_h
#define _ROS_map_msgs_ProjectedMap_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"
#include "nav_msgs/OccupancyGrid.h"

namespace map_msgs
{

  class ProjectedMap : public ros::Msg
  {
    public:
      typedef nav_msgs::OccupancyGrid _map_type;
      _map_type map;
      typedef float _min_z_type;
      _min_z_type min_z;
      typedef float _max_z_type;
      _max_z_type max_z;

    ProjectedMap():
      map(),
      min_z(0),
      max_z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->map.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->min_z);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->map.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->min_z));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_z));
     return offset;
    }

    #ifdef ESP8266
        const char * getType() { return  ("map_msgs/ProjectedMap");};
    #else
        const char * getType() { return  PSTR("map_msgs/ProjectedMap");};
    #endif
    #ifdef ESP8266
        const char * getMD5() { return  ("7bbe8f96e45089681dc1ea7d023cbfca");};
    #else
        const char * getMD5() { return  PSTR("7bbe8f96e45089681dc1ea7d023cbfca");};
    #endif

  };

}
#endif
