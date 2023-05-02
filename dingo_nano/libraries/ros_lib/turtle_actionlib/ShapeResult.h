#ifndef _ROS_turtle_actionlib_ShapeResult_h
#define _ROS_turtle_actionlib_ShapeResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace turtle_actionlib
{

  class ShapeResult : public ros::Msg
  {
    public:
      typedef float _interior_angle_type;
      _interior_angle_type interior_angle;
      typedef float _apothem_type;
      _apothem_type apothem;

    ShapeResult():
      interior_angle(0),
      apothem(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_interior_angle;
      u_interior_angle.real = this->interior_angle;
      *(outbuffer + offset + 0) = (u_interior_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_interior_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_interior_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_interior_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->interior_angle);
      union {
        float real;
        uint32_t base;
      } u_apothem;
      u_apothem.real = this->apothem;
      *(outbuffer + offset + 0) = (u_apothem.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_apothem.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_apothem.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_apothem.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->apothem);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_interior_angle;
      u_interior_angle.base = 0;
      u_interior_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_interior_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_interior_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_interior_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->interior_angle = u_interior_angle.real;
      offset += sizeof(this->interior_angle);
      union {
        float real;
        uint32_t base;
      } u_apothem;
      u_apothem.base = 0;
      u_apothem.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_apothem.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_apothem.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_apothem.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->apothem = u_apothem.real;
      offset += sizeof(this->apothem);
     return offset;
    }

    virtual const char * getType() override { return "turtle_actionlib/ShapeResult"; };
    virtual const char * getMD5() override { return "b06c6e2225f820dbc644270387cd1a7c"; };

  };

}
#endif
