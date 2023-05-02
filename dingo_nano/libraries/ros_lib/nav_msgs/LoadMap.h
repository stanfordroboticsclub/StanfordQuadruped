#ifndef _ROS_SERVICE_LoadMap_h
#define _ROS_SERVICE_LoadMap_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nav_msgs/OccupancyGrid.h"

namespace nav_msgs
{

static const char LOADMAP[] = "nav_msgs/LoadMap";

  class LoadMapRequest : public ros::Msg
  {
    public:
      typedef const char* _map_url_type;
      _map_url_type map_url;

    LoadMapRequest():
      map_url("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_map_url = strlen(this->map_url);
      varToArr(outbuffer + offset, length_map_url);
      offset += 4;
      memcpy(outbuffer + offset, this->map_url, length_map_url);
      offset += length_map_url;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_map_url;
      arrToVar(length_map_url, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_map_url; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_map_url-1]=0;
      this->map_url = (char *)(inbuffer + offset-1);
      offset += length_map_url;
     return offset;
    }

    virtual const char * getType() override { return LOADMAP; };
    virtual const char * getMD5() override { return "3813ba1ae85fbcd4dc88c90f1426b90b"; };

  };

  class LoadMapResponse : public ros::Msg
  {
    public:
      typedef nav_msgs::OccupancyGrid _map_type;
      _map_type map;
      typedef uint8_t _result_type;
      _result_type result;
      enum { RESULT_SUCCESS = 0 };
      enum { RESULT_MAP_DOES_NOT_EXIST = 1 };
      enum { RESULT_INVALID_MAP_DATA = 2 };
      enum { RESULT_INVALID_MAP_METADATA = 3 };
      enum { RESULT_UNDEFINED_FAILURE = 255 };

    LoadMapResponse():
      map(),
      result(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->map.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->result >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->map.deserialize(inbuffer + offset);
      this->result =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->result);
     return offset;
    }

    virtual const char * getType() override { return LOADMAP; };
    virtual const char * getMD5() override { return "079b9c828e9f7c1918bf86932fd7267e"; };

  };

  class LoadMap {
    public:
    typedef LoadMapRequest Request;
    typedef LoadMapResponse Response;
  };

}
#endif
