#ifndef _ROS_control_msgs_FollowJointTrajectoryActionResult_h
#define _ROS_control_msgs_FollowJointTrajectoryActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "control_msgs/FollowJointTrajectoryResult.h"

namespace control_msgs
{

  class FollowJointTrajectoryActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef control_msgs::FollowJointTrajectoryResult _result_type;
      _result_type result;

    FollowJointTrajectoryActionResult():
      header(),
      status(),
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    #ifdef ESP8266
        const char * getType() { return  ("control_msgs/FollowJointTrajectoryActionResult");};
    #else
        const char * getType() { return  PSTR("control_msgs/FollowJointTrajectoryActionResult");};
    #endif
    #ifdef ESP8266
        const char * getMD5() { return  ("c4fb3b000dc9da4fd99699380efcc5d9");};
    #else
        const char * getMD5() { return  PSTR("c4fb3b000dc9da4fd99699380efcc5d9");};
    #endif

  };

}
#endif
