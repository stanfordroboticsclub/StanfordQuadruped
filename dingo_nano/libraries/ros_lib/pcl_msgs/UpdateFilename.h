#ifndef _ROS_SERVICE_UpdateFilename_h
#define _ROS_SERVICE_UpdateFilename_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pcl_msgs
{

static const char UPDATEFILENAME[] = "pcl_msgs/UpdateFilename";

  class UpdateFilenameRequest : public ros::Msg
  {
    public:
      typedef const char* _filename_type;
      _filename_type filename;

    UpdateFilenameRequest():
      filename("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_filename = strlen(this->filename);
      varToArr(outbuffer + offset, length_filename);
      offset += 4;
      memcpy(outbuffer + offset, this->filename, length_filename);
      offset += length_filename;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_filename;
      arrToVar(length_filename, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_filename; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_filename-1]=0;
      this->filename = (char *)(inbuffer + offset-1);
      offset += length_filename;
     return offset;
    }

    virtual const char * getType() override { return UPDATEFILENAME; };
    virtual const char * getMD5() override { return "030824f52a0628ead956fb9d67e66ae9"; };

  };

  class UpdateFilenameResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    UpdateFilenameResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    virtual const char * getType() override { return UPDATEFILENAME; };
    virtual const char * getMD5() override { return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class UpdateFilename {
    public:
    typedef UpdateFilenameRequest Request;
    typedef UpdateFilenameResponse Response;
  };

}
#endif
