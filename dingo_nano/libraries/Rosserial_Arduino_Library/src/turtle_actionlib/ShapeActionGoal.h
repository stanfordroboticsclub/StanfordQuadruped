#ifndef _ROS_turtle_actionlib_ShapeActionGoal_h
#define _ROS_turtle_actionlib_ShapeActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "turtle_actionlib/ShapeGoal.h"

namespace turtle_actionlib
{

  class ShapeActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef turtle_actionlib::ShapeGoal _goal_type;
      _goal_type goal;

    ShapeActionGoal():
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
        const char * getType() { return  ("turtle_actionlib/ShapeActionGoal");};
    #else
        const char * getType() { return  PSTR("turtle_actionlib/ShapeActionGoal");};
    #endif
    #ifdef ESP8266
        const char * getMD5() { return  ("dbfccd187f2ec9c593916447ffd6cc77");};
    #else
        const char * getMD5() { return  PSTR("dbfccd187f2ec9c593916447ffd6cc77");};
    #endif

  };

}
#endif
