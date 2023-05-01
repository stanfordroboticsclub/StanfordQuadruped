#ifndef _ROS_SERVICE_GetPointMap_h
#define _ROS_SERVICE_GetPointMap_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"
#include "sensor_msgs/PointCloud2.h"

namespace map_msgs
{

#ifdef ESP8266
    static const char GETPOINTMAP[] = "map_msgs/GetPointMap";
#else
    static const char GETPOINTMAP[] PROGMEM = "map_msgs/GetPointMap";
#endif

  class GetPointMapRequest : public ros::Msg
  {
    public:

    GetPointMapRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return GETPOINTMAP; };
    #ifdef ESP8266
        const char * getMD5() { return  ("d41d8cd98f00b204e9800998ecf8427e");};
    #else
        const char * getMD5() { return  PSTR("d41d8cd98f00b204e9800998ecf8427e");};
    #endif

  };

  class GetPointMapResponse : public ros::Msg
  {
    public:
      typedef sensor_msgs::PointCloud2 _map_type;
      _map_type map;

    GetPointMapResponse():
      map()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->map.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->map.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETPOINTMAP; };
    #ifdef ESP8266
        const char * getMD5() { return  ("b84fbb39505086eb6a62d933c75cb7b4");};
    #else
        const char * getMD5() { return  PSTR("b84fbb39505086eb6a62d933c75cb7b4");};
    #endif

  };

  class GetPointMap {
    public:
    typedef GetPointMapRequest Request;
    typedef GetPointMapResponse Response;
  };

}
#endif
