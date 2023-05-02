#ifndef _ROS_gazebo_msgs_SensorPerformanceMetric_h
#define _ROS_gazebo_msgs_SensorPerformanceMetric_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gazebo_msgs
{

  class SensorPerformanceMetric : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef float _sim_update_rate_type;
      _sim_update_rate_type sim_update_rate;
      typedef float _real_update_rate_type;
      _real_update_rate_type real_update_rate;
      typedef float _fps_type;
      _fps_type fps;

    SensorPerformanceMetric():
      name(""),
      sim_update_rate(0),
      real_update_rate(0),
      fps(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      offset += serializeAvrFloat64(outbuffer + offset, this->sim_update_rate);
      offset += serializeAvrFloat64(outbuffer + offset, this->real_update_rate);
      offset += serializeAvrFloat64(outbuffer + offset, this->fps);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->sim_update_rate));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->real_update_rate));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->fps));
     return offset;
    }

    virtual const char * getType() override { return "gazebo_msgs/SensorPerformanceMetric"; };
    virtual const char * getMD5() override { return "01762ded18cfe9ebc7c8222667c99547"; };

  };

}
#endif
