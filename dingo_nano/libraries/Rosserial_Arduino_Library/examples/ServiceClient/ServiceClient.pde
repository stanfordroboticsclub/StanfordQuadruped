/*
 * rosserial Service Client
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <rosserial_arduino/Test.h>

ros::NodeHandle  nh;
using rosserial_arduino::Test;

ros::ServiceClient<Test::Request, Test::Response> client("test_srv");

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.serviceClient(client);
  nh.advertise(chatter);
  while(!nh.connected()) nh.spinOnce();
  nh.loginfo("Startup complete");
}

void loop()
{
  Test::Request req;
  Test::Response res;
  req.input = hello;
  client.call(req, res);
  str_msg.data = res.output;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(100);
}
