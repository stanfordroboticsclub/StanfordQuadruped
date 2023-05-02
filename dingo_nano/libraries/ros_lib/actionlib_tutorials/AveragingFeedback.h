#ifndef _ROS_actionlib_tutorials_AveragingFeedback_h
#define _ROS_actionlib_tutorials_AveragingFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace actionlib_tutorials
{

  class AveragingFeedback : public ros::Msg
  {
    public:
      typedef int32_t _sample_type;
      _sample_type sample;
      typedef float _data_type;
      _data_type data;
      typedef float _mean_type;
      _mean_type mean;
      typedef float _std_dev_type;
      _std_dev_type std_dev;

    AveragingFeedback():
      sample(0),
      data(0),
      mean(0),
      std_dev(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_sample;
      u_sample.real = this->sample;
      *(outbuffer + offset + 0) = (u_sample.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sample.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sample.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sample.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sample);
      union {
        float real;
        uint32_t base;
      } u_data;
      u_data.real = this->data;
      *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_data.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_data.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data);
      union {
        float real;
        uint32_t base;
      } u_mean;
      u_mean.real = this->mean;
      *(outbuffer + offset + 0) = (u_mean.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mean.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mean.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mean.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mean);
      union {
        float real;
        uint32_t base;
      } u_std_dev;
      u_std_dev.real = this->std_dev;
      *(outbuffer + offset + 0) = (u_std_dev.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_std_dev.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_std_dev.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_std_dev.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->std_dev);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_sample;
      u_sample.base = 0;
      u_sample.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sample.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sample.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sample.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sample = u_sample.real;
      offset += sizeof(this->sample);
      union {
        float real;
        uint32_t base;
      } u_data;
      u_data.base = 0;
      u_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data = u_data.real;
      offset += sizeof(this->data);
      union {
        float real;
        uint32_t base;
      } u_mean;
      u_mean.base = 0;
      u_mean.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mean.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mean.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mean.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mean = u_mean.real;
      offset += sizeof(this->mean);
      union {
        float real;
        uint32_t base;
      } u_std_dev;
      u_std_dev.base = 0;
      u_std_dev.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_std_dev.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_std_dev.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_std_dev.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->std_dev = u_std_dev.real;
      offset += sizeof(this->std_dev);
     return offset;
    }

    virtual const char * getType() override { return "actionlib_tutorials/AveragingFeedback"; };
    virtual const char * getMD5() override { return "9e8dfc53c2f2a032ca33fa80ec46fd4f"; };

  };

}
#endif
