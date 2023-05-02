#ifndef _ROS_actionlib_tutorials_AveragingActionGoal_h
#define _ROS_actionlib_tutorials_AveragingActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "actionlib_tutorials/AveragingGoal.h"

namespace actionlib_tutorials
{

  class AveragingActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef actionlib_tutorials::AveragingGoal _goal_type;
      _goal_type goal;

    AveragingActionGoal():
      header(),
      goal_id(),
      goal()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->goal_id.serialize(outbuffer + offset);
      offset += this->goal.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->goal_id.deserialize(inbuffer + offset);
      offset += this->goal.deserialize(inbuffer + offset);
     return offset;
    }

    #ifdef ESP8266
        const char * getType() { return  ("actionlib_tutorials/AveragingActionGoal");};
    #else
        const char * getType() { return  PSTR("actionlib_tutorials/AveragingActionGoal");};
    #endif
    #ifdef ESP8266
        const char * getMD5() { return  ("1561825b734ebd6039851c501e3fb570");};
    #else
        const char * getMD5() { return  PSTR("1561825b734ebd6039851c501e3fb570");};
    #endif

  };

}
#endif
