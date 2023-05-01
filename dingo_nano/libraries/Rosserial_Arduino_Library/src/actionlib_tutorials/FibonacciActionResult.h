#ifndef _ROS_actionlib_tutorials_FibonacciActionResult_h
#define _ROS_actionlib_tutorials_FibonacciActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "actionlib_tutorials/FibonacciResult.h"

namespace actionlib_tutorials
{

  class FibonacciActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef actionlib_tutorials::FibonacciResult _result_type;
      _result_type result;

    FibonacciActionResult():
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
        const char * getType() { return  ("actionlib_tutorials/FibonacciActionResult");};
    #else
        const char * getType() { return  PSTR("actionlib_tutorials/FibonacciActionResult");};
    #endif
    #ifdef ESP8266
        const char * getMD5() { return  ("bee73a9fe29ae25e966e105f5553dd03");};
    #else
        const char * getMD5() { return  PSTR("bee73a9fe29ae25e966e105f5553dd03");};
    #endif

  };

}
#endif
