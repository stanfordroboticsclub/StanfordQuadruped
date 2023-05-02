#ifndef _ROS_control_msgs_SingleJointPositionActionResult_h
#define _ROS_control_msgs_SingleJointPositionActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "control_msgs/SingleJointPositionResult.h"

namespace control_msgs
{

  class SingleJointPositionActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef control_msgs::SingleJointPositionResult _result_type;
      _result_type result;

    SingleJointPositionActionResult():
      header(),
      status(),
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "control_msgs/SingleJointPositionActionResult"; };
    virtual const char * getMD5() override { return "1eb06eeff08fa7ea874431638cb52332"; };

  };

}
#endif
