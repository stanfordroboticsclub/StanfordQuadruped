#ifndef _ROS_SERVICE_ProjectedMapsInfo_h
#define _ROS_SERVICE_ProjectedMapsInfo_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "map_msgs/ProjectedMapInfo.h"

namespace map_msgs
{

static const char PROJECTEDMAPSINFO[] = "map_msgs/ProjectedMapsInfo";

  class ProjectedMapsInfoRequest : public ros::Msg
  {
    public:
      uint32_t projected_maps_info_length;
      typedef map_msgs::ProjectedMapInfo _projected_maps_info_type;
      _projected_maps_info_type st_projected_maps_info;
      _projected_maps_info_type * projected_maps_info;

    ProjectedMapsInfoRequest():
      projected_maps_info_length(0), st_projected_maps_info(), projected_maps_info(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->projected_maps_info_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->projected_maps_info_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->projected_maps_info_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->projected_maps_info_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->projected_maps_info_length);
      for( uint32_t i = 0; i < projected_maps_info_length; i++){
      offset += this->projected_maps_info[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t projected_maps_info_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      projected_maps_info_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      projected_maps_info_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      projected_maps_info_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->projected_maps_info_length);
      if(projected_maps_info_lengthT > projected_maps_info_length)
        this->projected_maps_info = (map_msgs::ProjectedMapInfo*)realloc(this->projected_maps_info, projected_maps_info_lengthT * sizeof(map_msgs::ProjectedMapInfo));
      projected_maps_info_length = projected_maps_info_lengthT;
      for( uint32_t i = 0; i < projected_maps_info_length; i++){
      offset += this->st_projected_maps_info.deserialize(inbuffer + offset);
        memcpy( &(this->projected_maps_info[i]), &(this->st_projected_maps_info), sizeof(map_msgs::ProjectedMapInfo));
      }
     return offset;
    }

    virtual const char * getType() override { return PROJECTEDMAPSINFO; };
    virtual const char * getMD5() override { return "d7980a33202421c8cd74565e57a4d229"; };

  };

  class ProjectedMapsInfoResponse : public ros::Msg
  {
    public:

    ProjectedMapsInfoResponse()
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

    virtual const char * getType() override { return PROJECTEDMAPSINFO; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ProjectedMapsInfo {
    public:
    typedef ProjectedMapsInfoRequest Request;
    typedef ProjectedMapsInfoResponse Response;
  };

}
#endif
