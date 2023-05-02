#ifndef _ROS_turtle_actionlib_ShapeGoal_h
#define _ROS_turtle_actionlib_ShapeGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace turtle_actionlib
{

  class ShapeGoal : public ros::Msg
  {
    public:
      typedef int32_t _edges_type;
      _edges_type edges;
      typedef float _radius_type;
      _radius_type radius;

    ShapeGoal():
      edges(0),
      radius(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_edges;
      u_edges.real = this->edges;
      *(outbuffer + offset + 0) = (u_edges.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_edges.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_edges.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_edges.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->edges);
      union {
        float real;
        uint32_t base;
      } u_radius;
      u_radius.real = this->radius;
      *(outbuffer + offset + 0) = (u_radius.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_radius.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_radius.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_radius.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->radius);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_edges;
      u_edges.base = 0;
      u_edges.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_edges.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_edges.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_edges.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->edges = u_edges.real;
      offset += sizeof(this->edges);
      union {
        float real;
        uint32_t base;
      } u_radius;
      u_radius.base = 0;
      u_radius.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_radius.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_radius.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_radius.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->radius = u_radius.real;
      offset += sizeof(this->radius);
     return offset;
    }

    virtual const char * getType() override { return "turtle_actionlib/ShapeGoal"; };
    virtual const char * getMD5() override { return "3b9202ab7292cebe5a95ab2bf6b9c091"; };

  };

}
#endif
