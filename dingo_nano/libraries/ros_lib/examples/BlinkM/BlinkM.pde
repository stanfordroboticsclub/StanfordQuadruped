/*
*  RosSerial BlinkM Example
*  This program shows how to control a blinkm
*  from an arduino using RosSerial
*/

#include <stdlib.h>


#include <ros.h>
#include <std_msgs/String.h>


//include Wire/ twi for the BlinkM
#include <Wire.h>
extern "C" { 
#include "utility/twi.h" 
}

#include "BlinkM_funcs.h"
const byte blinkm_addr = 0x09; //default blinkm address


void setLED( bool solid,  char color)
{

       	if (solid)
	{
           switch (color)
		{

		case 'w':  // white
			BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0xff,0xff,0xff);  
			break;
			
		case 'r': //RED
			BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0xff,0,0);  
			break;

		case 'g':// Green
			BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0,0xff,0);
			break;

		case 'b':// Blue
			BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0,0,0xff);
			break;

		case 'c':// Cyan
			BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0,0xff,0xff);
			break;

		case 'm': // Magenta
			BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0xff,0,0xff);
			break;

		case 'y': // yellow
                        BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0xff,0xff,0);
			break;

		default: // Black
			BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0,0,0);
			break;
		}
	}


	else
	{
               	switch (color)
		{
		case 'r':  // Blink Red
                        BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 3,0,0 );
			break;
		case 'w':  // Blink white
                        BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 2,0,0 );
			break;
		case 'g':  // Blink Green
			BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 4,0,0 );
			break;

		case 'b': // Blink Blue
			BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 5,0,0 );
			break;

		case 'c': //Blink Cyan
			BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 6,0,0 );
			break;

		case 'm': //Blink Magenta
			BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 7,0,0 );
			break;

		case 'y': //Blink Yellow
                        BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 8,0,0 );
			break;

		default: //OFF
			BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 9,0,0 );
			break;
		}

	}
}

void light_cb( const std_msgs::String& light_cmd){
        bool solid =false;
        char color; 
        if (strlen( (const char* ) light_cmd.data) ==2 ){
          solid  = (light_cmd.data[0] == 'S') || (light_cmd.data[0] == 's');
          color = light_cmd.data[1];
        }
        else{
          solid=  false;
          color = light_cmd.data[0];
        } 
        
	setLED(solid, color);
}



ros::NodeHandle  nh;
ros::Subscriber<std_msgs::String> sub("blinkm" , light_cb);


void setup()
{
   
    pinMode(13, OUTPUT); //set up the LED

	BlinkM_beginWithPower();
	delay(100);
	BlinkM_stopScript(blinkm_addr);  // turn off startup script
	setLED(false, 0); //turn off the led
        
        nh.initNode();
        nh.subscribe(sub);

}

void loop()
{
 nh.spinOnce();
 delay(1);
}

