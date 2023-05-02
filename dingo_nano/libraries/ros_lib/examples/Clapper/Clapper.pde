/* 
 * rosserial Clapper Example
 * 
 * This code is a very simple example of the kinds of 
 * custom sensors that you can easily set up with rosserial
 * and Arduino.  This code uses a microphone attached to 
 * analog pin 5 detect two claps (2 loud sounds).
 * You can use this clapper, for example, to command a robot 
 * in the area to come do your bidding.
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;

std_msgs::Empty clap_msg;
ros::Publisher p("clap", &clap_msg);

enum clapper_state { clap1, clap_one_waiting,  pause, clap2};
clapper_state clap;

int volume_thresh = 200;  //a clap sound needs to be: 
                          //abs(clap_volume) > average noise + volume_thresh
int mic_pin = 5;
int adc_ave=0;

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();

  nh.advertise(p);

  //measure the average volume of the noise in the area
  for (int i =0; i<10;i++) adc_ave += analogRead(mic_pin);
  adc_ave /= 10;
}

long event_timer;

void loop()
{
  int mic_val = 0;
  for(int i=0; i<4; i++) mic_val += analogRead(mic_pin);
  
  mic_val = mic_val/4-adc_ave;
  
  switch(clap){
   case clap1:
    if (abs(mic_val) > volume_thresh){
       clap = clap_one_waiting; 
       event_timer = millis();
    }
    break;
    case clap_one_waiting:
      if ( (abs(mic_val) < 30) && ( (millis() - event_timer) > 20 ) )  
      {
        clap= pause;
        event_timer = millis();

      }
      break;
    case pause: // make sure there is a pause between 
                // the loud sounds
      if ( mic_val > volume_thresh) 
      {
        clap = clap1;

      }
      else if ( (millis()-event_timer)> 60)  {
        clap = clap2;
        event_timer = millis();

      }
      break;
     case clap2:
        if (abs(mic_val) > volume_thresh){ //we have got a double clap!
            clap = clap1;
            p.publish(&clap_msg);
        }
        else if ( (millis()-event_timer)> 200) {
          clap= clap1; // no clap detected, reset state machine
        }
        
        break;   
  }
  nh.spinOnce();
}
