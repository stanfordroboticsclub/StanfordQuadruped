#ifndef _ROS_control_msgs_SingleJointPositionActionGoal_h
#define _ROS_control_msgs_SingleJointPositionActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "control_msgs/SingleJointPositionGoal.h"

namespace control_msgs
{

  class SingleJointPositionActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef control_msgs::SingleJointPositionGoal _goal_type;
      _goal_type goal;

    SingleJointPositionActionGoal():
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
        const char * getType() { return  ("control_msgs/SingleJointPositionActionGoal");};
    #else
        const char * getType() { return  PSTR("control_msgs/SingleJointPositionActionGoal");};
    #endif
    #ifdef ESP8266
        const char * getMD5() { return  ("4b0d3d091471663e17749c1d0db90f61");};
    #else
        const char * getMD5() { return  PSTR("4b0d3d091471663e17749c1d0db90f61");};
    #endif

  };

}
#endif
