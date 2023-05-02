/*
 * rosserial Temperature Sensor Example
 * 
 * This tutorial demonstrates the usage of the
 * Sparkfun TMP102 Digital Temperature Breakout board
 * http://www.sparkfun.com/products/9418
 * 
 * Source Code Based off of:
 * http://wiring.org.co/learning/libraries/tmp102sparkfun.html
 */

#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;


std_msgs::Float32 temp_msg;
ros::Publisher pub_temp("temperature", &temp_msg);


// From the datasheet the BMP module address LSB distinguishes
// between read (1) and write (0) operations, corresponding to 
// address 0x91 (read) and 0x90 (write).
// shift the address 1 bit right (0x91 or 0x90), the Wire library only needs the 7
// most significant bits for the address 0x91 >> 1 = 0x48
// 0x90 >> 1 = 0x48 (72)

int sensorAddress = 0x91 >> 1;  // From datasheet sensor address is 0x91
                                // shift the address 1 bit right, the Wire library only needs the 7
                                // most significant bits for the address


void setup()
{
  Wire.begin();        // join i2c bus (address optional for master) 
  
  nh.initNode();
  nh.advertise(pub_temp);
  
}

long publisher_timer;

void loop()
{
  
  if (millis() > publisher_timer) {
  // step 1: request reading from sensor 
    Wire.requestFrom(sensorAddress,2); 
    delay(10);
    if (2 <= Wire.available())  // if two bytes were received 
    {
      byte msb;
      byte lsb;
      int temperature;
      
      msb = Wire.read();  // receive high byte (full degrees)
      lsb = Wire.read();  // receive low byte (fraction degrees) 
      temperature = ((msb) << 4);  // MSB
      temperature |= (lsb >> 4);   // LSB

      temp_msg.data = temperature*0.0625;
      pub_temp.publish(&temp_msg);
    }
  
  publisher_timer = millis() + 1000;
  }
  
  nh.spinOnce();
}
