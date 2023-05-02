#ifndef _ROS_pcl_msgs_PolygonMesh_h
#define _ROS_pcl_msgs_PolygonMesh_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_msgs/Vertices.h"

namespace pcl_msgs
{

  class PolygonMesh : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef sensor_msgs::PointCloud2 _cloud_type;
      _cloud_type cloud;
      uint32_t polygons_length;
      typedef pcl_msgs::Vertices _polygons_type;
      _polygons_type st_polygons;
      _polygons_type * polygons;

    PolygonMesh():
      header(),
      cloud(),
      polygons_length(0), st_polygons(), polygons(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->cloud.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->polygons_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->polygons_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->polygons_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->polygons_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->polygons_length);
      for( uint32_t i = 0; i < polygons_length; i++){
      offset += this->polygons[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->cloud.deserialize(inbuffer + offset);
      uint32_t polygons_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      polygons_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      polygons_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      polygons_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->polygons_length);
      if(polygons_lengthT > polygons_length)
        this->polygons = (pcl_msgs::Vertices*)realloc(this->polygons, polygons_lengthT * sizeof(pcl_msgs::Vertices));
      polygons_length = polygons_lengthT;
      for( uint32_t i = 0; i < polygons_length; i++){
      offset += this->st_polygons.deserialize(inbuffer + offset);
        memcpy( &(this->polygons[i]), &(this->st_polygons), sizeof(pcl_msgs::Vertices));
      }
     return offset;
    }

    virtual const char * getType() override { return "pcl_msgs/PolygonMesh"; };
    virtual const char * getMD5() override { return "45a5fc6ad2cde8489600a790acc9a38a"; };

  };

}
#endif
