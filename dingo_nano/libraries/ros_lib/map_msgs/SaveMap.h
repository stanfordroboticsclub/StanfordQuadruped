#ifndef _ROS_SERVICE_SaveMap_h
#define _ROS_SERVICE_SaveMap_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/String.h"

namespace map_msgs
{

static const char SAVEMAP[] = "map_msgs/SaveMap";

  class SaveMapRequest : public ros::Msg
  {
    public:
      typedef std_msgs::String _filename_type;
      _filename_type filename;

    SaveMapRequest():
      filename()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->filename.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->filename.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SAVEMAP; };
    virtual const char * getMD5() override { return "716e25f9d9dc76ceba197f93cbf05dc7"; };

  };

  class SaveMapResponse : public ros::Msg
  {
    public:

    SaveMapResponse()
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

    virtual const char * getType() override { return SAVEMAP; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SaveMap {
    public:
    typedef SaveMapRequest Request;
    typedef SaveMapResponse Response;
  };

}
#endif
