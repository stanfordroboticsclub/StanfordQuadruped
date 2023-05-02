#ifndef _ROS_control_msgs_JointTrajectoryActionGoal_h
#define _ROS_control_msgs_JointTrajectoryActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "control_msgs/JointTrajectoryGoal.h"

namespace control_msgs
{

  class JointTrajectoryActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef control_msgs::JointTrajectoryGoal _goal_type;
      _goal_type goal;

    JointTrajectoryActionGoal():
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
        const char * getType() { return  ("control_msgs/JointTrajectoryActionGoal");};
    #else
        const char * getType() { return  PSTR("control_msgs/JointTrajectoryActionGoal");};
    #endif
    #ifdef ESP8266
        const char * getMD5() { return  ("a99e83ef6185f9fdd7693efe99623a86");};
    #else
        const char * getMD5() { return  PSTR("a99e83ef6185f9fdd7693efe99623a86");};
    #endif

  };

}
#endif
