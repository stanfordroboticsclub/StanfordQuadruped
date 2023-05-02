#ifndef _ROS_shape_msgs_Mesh_h
#define _ROS_shape_msgs_Mesh_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "shape_msgs/MeshTriangle.h"
#include "geometry_msgs/Point.h"

namespace shape_msgs
{

  class Mesh : public ros::Msg
  {
    public:
      uint32_t triangles_length;
      typedef shape_msgs::MeshTriangle _triangles_type;
      _triangles_type st_triangles;
      _triangles_type * triangles;
      uint32_t vertices_length;
      typedef geometry_msgs::Point _vertices_type;
      _vertices_type st_vertices;
      _vertices_type * vertices;

    Mesh():
      triangles_length(0), st_triangles(), triangles(nullptr),
      vertices_length(0), st_vertices(), vertices(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->triangles_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->triangles_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->triangles_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->triangles_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->triangles_length);
      for( uint32_t i = 0; i < triangles_length; i++){
      offset += this->triangles[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->vertices_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vertices_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->vertices_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->vertices_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vertices_length);
      for( uint32_t i = 0; i < vertices_length; i++){
      offset += this->vertices[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t triangles_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      triangles_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      triangles_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      triangles_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->triangles_length);
      if(triangles_lengthT > triangles_length)
        this->triangles = (shape_msgs::MeshTriangle*)realloc(this->triangles, triangles_lengthT * sizeof(shape_msgs::MeshTriangle));
      triangles_length = triangles_lengthT;
      for( uint32_t i = 0; i < triangles_length; i++){
      offset += this->st_triangles.deserialize(inbuffer + offset);
        memcpy( &(this->triangles[i]), &(this->st_triangles), sizeof(shape_msgs::MeshTriangle));
      }
      uint32_t vertices_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      vertices_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      vertices_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      vertices_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->vertices_length);
      if(vertices_lengthT > vertices_length)
        this->vertices = (geometry_msgs::Point*)realloc(this->vertices, vertices_lengthT * sizeof(geometry_msgs::Point));
      vertices_length = vertices_lengthT;
      for( uint32_t i = 0; i < vertices_length; i++){
      offset += this->st_vertices.deserialize(inbuffer + offset);
        memcpy( &(this->vertices[i]), &(this->st_vertices), sizeof(geometry_msgs::Point));
      }
     return offset;
    }

    virtual const char * getType() override { return "shape_msgs/Mesh"; };
    virtual const char * getMD5() override { return "1ffdae9486cd3316a121c578b47a85cc"; };

  };

}
#endif
