/* 
 * rosserial::std_msgs::Time Test
 * Publishes current time
 */

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Time.h>


ros::NodeHandle  nh;

std_msgs::Time test;
ros::Publisher p("my_topic", &test);

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(p);
}

void loop()
{  
  test.data = nh.now();
  p.publish( &test );
  nh.spinOnce();
  delay(10);
}

