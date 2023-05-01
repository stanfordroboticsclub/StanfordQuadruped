/* 
 * rosserial Ultrasound Example
 * 
 * This example is for the Maxbotix Ultrasound rangers.
 */

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);

const int adc_pin = 0;

char frameid[] = "/ultrasound";

float getRange_Ultrasound(int pin_num){
  int val = 0;
  for(int i=0; i<4; i++) val += analogRead(pin_num);
  float range =  val;
  return range /322.519685;   // (0.0124023437 /4) ; //cvt to meters
}

void setup()
{
  nh.initNode();
  nh.advertise(pub_range);
  
  
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.0;
  range_msg.max_range = 6.47;
  
  pinMode(8,OUTPUT);
  digitalWrite(8, LOW);
}


long range_time;

void loop()
{
  
  //publish the adc value every 50 milliseconds
  //since it takes that long for the sensor to stablize
  if ( millis() >= range_time ){
    int r =0;

    range_msg.range = getRange_Ultrasound(5);
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    range_time =  millis() + 50;
  }
  
  nh.spinOnce();
}
