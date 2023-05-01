/*
 * BlinkM_funcs.h -- Arduino 'library' to control BlinkM
 * --------------
 *
 *
 * Note: original version of this file lives with the BlinkMTester sketch
 *
 * Note: all the functions are declared 'static' because 
 *       it saves about 1.5 kbyte in code space in final compiled sketch.  
 *       A C++ library of this costs a 1kB more.
 *
 * 2007-8, Tod E. Kurt, ThingM, http://thingm.com/
 *
 * version: 20081101
 *
 * history:
 *  20080101 - initial release
 *  20080203 - added setStartupParam(), bugfix receiveBytes() from Dan Julio
 *  20081101 - fixed to work with Arduino-0012, added MaxM commands,
 *             added test script read/write functions, cleaned up some functions
 *  20090121 - added I2C bus scan functions, has dependencies on private 
 *             functions inside Wire library, so might break in the future
 *  20100420 - added BlinkM_startPower and _stopPower
 *
 */

#include <Wire.h>

extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}


// format of light script lines: duration, command, arg1,arg2,arg3
typedef struct _blinkm_script_line {
  uint8_t dur;
  uint8_t cmd[4];    // cmd,arg1,arg2,arg3
} blinkm_script_line;


// Call this first (when powering BlinkM from a power supply)
static void BlinkM_begin()
{
  Wire.begin();                // join i2c bus (address optional for master)
}

/*
 * actually can't do this either, because twi_init() has THREE callocs in it too
 *
static void BlinkM_reset()
{
  twi_init();  // can't just call Wire.begin() again because of calloc()s there
}
*/

//
// each call to twi_writeTo() should return 0 if device is there
// or other value (usually 2) if nothing is at that address
// 
static void BlinkM_scanI2CBus(byte from, byte to, 
                              void(*callback)(byte add, byte result) ) 
{
  byte rc;
  byte data = 0; // not used, just an address to feed to twi_writeTo()
  for( byte addr = from; addr <= to; addr++ ) {
    rc = twi_writeTo(addr, &data, 0, 1, 1);
    callback( addr, rc );
  }
}

//
//
static int8_t BlinkM_findFirstI2CDevice() 
{
  byte rc;
  byte data = 0; // not used, just an address to feed to twi_writeTo()
  for( byte addr=1; addr < 120; addr++ ) {  // only scan addrs 1-120
    rc = twi_writeTo(addr, &data, 0, 1, 1);
    if( rc == 0 ) return addr; // found an address
  }
  return -1; // no device found in range given
}

// FIXME: make this more Arduino-like
static void BlinkM_startPowerWithPins(byte pwrpin, byte gndpin)
{
  DDRC |= _BV(pwrpin) | _BV(gndpin);  // make outputs
  PORTC &=~ _BV(gndpin);
  PORTC |=  _BV(pwrpin);
}

// FIXME: make this more Arduino-like
static void BlinkM_stopPowerWithPins(byte pwrpin, byte gndpin)
{
  DDRC &=~ (_BV(pwrpin) | _BV(gndpin));
}

//
static void BlinkM_startPower()
{
  BlinkM_startPowerWithPins( PORTC3, PORTC2 );
}

//
static void BlinkM_stopPower()
{
  BlinkM_stopPowerWithPins( PORTC3, PORTC2 );
}

// General version of BlinkM_beginWithPower().
// Call this first when BlinkM is plugged directly into Arduino
static void BlinkM_beginWithPowerPins(byte pwrpin, byte gndpin)
{
  BlinkM_startPowerWithPins(pwrpin,gndpin);
  delay(100);  // wait for things to stabilize
  Wire.begin();
}

// Call this first when BlinkM is plugged directly into Arduino
// FIXME: make this more Arduino-like
static void BlinkM_beginWithPower()
{
  BlinkM_beginWithPowerPins( PORTC3, PORTC2 );
}

// sends a generic command
static void BlinkM_sendCmd(byte addr, byte* cmd, int cmdlen)
{
  Wire.beginTransmission(addr);
  for( byte i=0; i<cmdlen; i++) 
    Wire.write(cmd[i]);
  Wire.endTransmission();
}

// receives generic data
// returns 0 on success, and -1 if no data available
// note: responsiblity of caller to know how many bytes to expect
static int BlinkM_receiveBytes(byte addr, byte* resp, byte len)
{
  Wire.requestFrom(addr, len);
  if( Wire.available() ) {
    for( int i=0; i<len; i++) 
      resp[i] = Wire.read();
    return 0;
  }
  return -1;
}

// Sets the I2C address of the BlinkM.  
// Uses "general call" broadcast address
static void BlinkM_setAddress(byte newaddress)
{
  Wire.beginTransmission(0x00);  // general call (broadcast address)
  Wire.write('A');
  Wire.write(newaddress);
  Wire.write(0xD0);
  Wire.write(0x0D);  // dood!
  Wire.write(newaddress);
  Wire.endTransmission();
  delay(50); // just in case
}


// Gets the I2C address of the BlinKM
// Kind of redundant when sent to a specific address
// but uses to verify BlinkM communication
static int BlinkM_getAddress(byte addr)
{
  Wire.beginTransmission(addr);
  Wire.write('a');
  Wire.endTransmission();
  Wire.requestFrom(addr, (byte)1);  // general call
  if( Wire.available() ) {
    byte b = Wire.read();
    return b;
  }
  return -1;
}

// Gets the BlinkM firmware version
static int BlinkM_getVersion(byte addr)
{
  Wire.beginTransmission(addr);
  Wire.write('Z');
  Wire.endTransmission();
  Wire.requestFrom(addr, (byte)2);
  if( Wire.available() ) {
    byte major_ver = Wire.read();
    byte minor_ver = Wire.read();
    return (major_ver<<8) + minor_ver;
  }
  return -1;
}

// Demonstrates how to verify you're talking to a BlinkM 
// and that you know its address
static int BlinkM_checkAddress(byte addr)
{
  //Serial.print("Checking BlinkM address...");
  int b = BlinkM_getAddress(addr);
  if( b==-1 ) {
    //Serial.println("No response, that's not good");
    return -1;  // no response
  } 
  //Serial.print("received addr: 0x");
  //Serial.print(b,HEX);
  if( b != addr )
    return 1; // error, addr mismatch 
  else 
    return 0; // match, everything okay
}

// Sets the speed of fading between colors.  
// Higher numbers means faster fading, 255 == instantaneous fading
static void BlinkM_setFadeSpeed(byte addr, byte fadespeed)
{
  Wire.beginTransmission(addr);
  Wire.write('f');
  Wire.write(fadespeed);
  Wire.endTransmission();  
}

// Sets the light script playback time adjust
// The timeadj argument is signed, and is an additive value to all
// durations in a light script. Set to zero to turn off time adjust.
static void BlinkM_setTimeAdj(byte addr, byte timeadj)
{
  Wire.beginTransmission(addr);
  Wire.write('t');
  Wire.write(timeadj);
  Wire.endTransmission();  
}

// Fades to an RGB color
static void BlinkM_fadeToRGB(byte addr, byte red, byte grn, byte blu)
{
  Wire.beginTransmission(addr);
  Wire.write('c');
  Wire.write(red);
  Wire.write(grn);
  Wire.write(blu);
  Wire.endTransmission();
}

// Fades to an HSB color
static void BlinkM_fadeToHSB(byte addr, byte hue, byte saturation, byte brightness)
{
  Wire.beginTransmission(addr);
  Wire.write('h');
  Wire.write(hue);
  Wire.write(saturation);
  Wire.write(brightness);
  Wire.endTransmission();
}

// Sets an RGB color immediately
static void BlinkM_setRGB(byte addr, byte red, byte grn, byte blu)
{
  Wire.beginTransmission(addr);
  Wire.write('n');
  Wire.write(red);
  Wire.write(grn);
  Wire.write(blu);
  Wire.endTransmission();
}

// Fades to a random RGB color
static void BlinkM_fadeToRandomRGB(byte addr, byte rrnd, byte grnd, byte brnd)
{
  Wire.beginTransmission(addr);
  Wire.write('C');
  Wire.write(rrnd);
  Wire.write(grnd);
  Wire.write(brnd);
  Wire.endTransmission();
}
// Fades to a random HSB color
static void BlinkM_fadeToRandomHSB(byte addr, byte hrnd, byte srnd, byte brnd)
{
  Wire.beginTransmission(addr);
  Wire.write('H');
  Wire.write(hrnd);
  Wire.write(srnd);
  Wire.write(brnd);
  Wire.endTransmission();
}

//
static void BlinkM_getRGBColor(byte addr, byte* r, byte* g, byte* b)
{
  Wire.beginTransmission(addr);
  Wire.write('g');
  Wire.endTransmission();
  Wire.requestFrom(addr, (byte)3);
  if( Wire.available() ) {
    *r = Wire.read();
    *g = Wire.read();
    *b = Wire.read();
  }
}

//
static void BlinkM_playScript(byte addr, byte script_id, byte reps, byte pos)
{
  Wire.beginTransmission(addr);
  Wire.write('p');
  Wire.write(script_id);
  Wire.write(reps);
  Wire.write(pos);
  Wire.endTransmission();
}

//
static void BlinkM_stopScript(byte addr)
{
  Wire.beginTransmission(addr);
  Wire.write('o');
  Wire.endTransmission();
}

//
static void BlinkM_setScriptLengthReps(byte addr, byte script_id, 
                                       byte len, byte reps)
{
  Wire.beginTransmission(addr);
  Wire.write('L');
  Wire.write(script_id);
  Wire.write(len);
  Wire.write(reps);
  Wire.endTransmission();
}

// Fill up script_line with data from a script line
// currently only script_id = 0 works (eeprom script)
static void BlinkM_readScriptLine(byte addr, byte script_id, 
                                  byte pos, blinkm_script_line* script_line)
{
  Wire.beginTransmission(addr);
  Wire.write('R');
  Wire.write(script_id);
  Wire.write(pos);
  Wire.endTransmission();
  Wire.requestFrom(addr, (byte)5);
  while( Wire.available() < 5 ) ; // FIXME: wait until we get 7 bytes
  script_line->dur    = Wire.read();
  script_line->cmd[0] = Wire.read();
  script_line->cmd[1] = Wire.read();
  script_line->cmd[2] = Wire.read();
  script_line->cmd[3] = Wire.read();
}

//
static void BlinkM_writeScriptLine(byte addr, byte script_id, 
                                   byte pos, byte dur,
                                   byte cmd, byte arg1, byte arg2, byte arg3)
{
#ifdef BLINKM_FUNCS_DEBUG
  Serial.print("writing line:");  Serial.print(pos,DEC);
  Serial.print(" with cmd:"); Serial.print(cmd); 
  Serial.print(" arg1:"); Serial.println(arg1,HEX);
#endif
  Wire.beginTransmission(addr);
  Wire.write('W');
  Wire.write(script_id);
  Wire.write(pos);
  Wire.write(dur);
  Wire.write(cmd);
  Wire.write(arg1);
  Wire.write(arg2);
  Wire.write(arg3);
  Wire.endTransmission();

}

//
static void BlinkM_writeScript(byte addr, byte script_id, 
                               byte len, byte reps,
                               blinkm_script_line* lines)
{
#ifdef BLINKM_FUNCS_DEBUG
  Serial.print("writing script to addr:"); Serial.print(addr,DEC);
  Serial.print(", script_id:"); Serial.println(script_id,DEC);
#endif
  for(byte i=0; i < len; i++) {
    blinkm_script_line l = lines[i];
    BlinkM_writeScriptLine( addr, script_id, i, l.dur,
                            l.cmd[0], l.cmd[1], l.cmd[2], l.cmd[3]);
    delay(20); // must wait for EEPROM to be programmed
  }
  BlinkM_setScriptLengthReps(addr, script_id, len, reps);
}

//
static void BlinkM_setStartupParams(byte addr, byte mode, byte script_id,
                                    byte reps, byte fadespeed, byte timeadj)
{
  Wire.beginTransmission(addr);
  Wire.write('B');
  Wire.write(mode);             // default 0x01 == Play script
  Wire.write(script_id);        // default 0x00 == script #0
  Wire.write(reps);             // default 0x00 == repeat infinitely
  Wire.write(fadespeed);        // default 0x08 == usually overridden by sketch 
  Wire.write(timeadj);          // default 0x00 == sometimes overridden by sketch
  Wire.endTransmission();
} 


// Gets digital inputs of the BlinkM
// returns -1 on failure
static int BlinkM_getInputsO(byte addr)
{
  Wire.beginTransmission(addr);
  Wire.write('i');
  Wire.endTransmission();
  Wire.requestFrom(addr, (byte)1);
  if( Wire.available() ) {
    byte b = Wire.read();
    return b; 
  }
  return -1;
}

// Gets digital inputs of the BlinkM
// stores them in passed in array
// returns -1 on failure
static int BlinkM_getInputs(byte addr, byte inputs[])
{
  Wire.beginTransmission(addr);
  Wire.write('i');
  Wire.endTransmission();
  Wire.requestFrom(addr, (byte)4);
  while( Wire.available() < 4 ) ; // FIXME: wait until we get 4 bytes
    
  inputs[0] = Wire.read();
  inputs[1] = Wire.read();
  inputs[2] = Wire.read();
  inputs[3] = Wire.read();

  return 0;
}
