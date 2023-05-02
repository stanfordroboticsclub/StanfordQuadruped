#ifndef _ROS_tf2_msgs_LookupTransformActionFeedback_h
#define _ROS_tf2_msgs_LookupTransformActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "tf2_msgs/LookupTransformFeedback.h"

namespace tf2_msgs
{

  class LookupTransformActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef tf2_msgs::LookupTransformFeedback _feedback_type;
      _feedback_type feedback;

    LookupTransformActionFeedback():
      header(),
      status(),
      feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->feedback.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "tf2_msgs/LookupTransformActionFeedback"; };
    virtual const char * getMD5() override { return "aae20e09065c3809e8a8e87c4c8953fd"; };

  };

}
#endif
