#include <ros.h>
#include <std_msgs/float32.h>
#include <avr/sleep.h>

ros::NodeHandle arduino_nano;
std_msgs::Float32 battery_voltage_msg;

ros::Publisher battery_voltage_publisher("battery_voltage_level", &battery_voltage_msg);

const int number_stored_voltages = 20;
float battery_voltage_array[number_stored_voltages];
float servo_buck_voltage_array[number_stored_voltages];
int current_voltage_index = 0;
int number_of_low_battery_detections = 0;
int number_of_0v_detections = 0;

void setup()
{
  arduino_nano.initNode();
  arduino_nano.advertise(battery_voltage_publisher);
}

void shutdown()
{
  //shuts the arduino down if the voltage level is too low
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
  sleep_enable();
  sleep_cpu ();             
}

void checkBatteryVoltageLevel()
{
  //retrieves the current voltage of the battery. Uses this and previous voltages to calculate the average voltage of the battery, then publishes to pi
  //Will shutdown the arduino if the voltage is too low
  const int battery_R1 = 27000;
  const int battery_R2 = 10000;
  const int minimum_battery_voltage = 15.5;
  
  int sensorValue = analogRead(A0);
  
  float current_voltage = sensorValue * (5.0 / 1023.0) * ((battery_R1+battery_R2)/battery_R2);
  battery_voltage_array[current_voltage_index] = current_voltage;

  float average_voltage = 0;
  int numberOfVoltagesUsed = 0;

  for (int j = 0; j < number_stored_voltages; j++)
  {
    if (battery_voltage_array[j] != 0.0)
    {
      average_voltage = average_voltage + battery_voltage_array[j];
      numberOfVoltagesUsed = numberOfVoltagesUsed + 1;
    }
      
  } 

  float current_battery_voltage = average_voltage/numberOfVoltagesUsed;
  battery_voltage_msg.data = current_battery_voltage;
  battery_voltage_publisher.publish(&battery_voltage_msg);

  if (current_battery_voltage < minimum_battery_voltage)
  {
    number_of_low_battery_detections = number_of_low_battery_detections + 1;
    if (number_of_low_battery_detections > 30)
    {
      shutdown();
    }
  }
  else if (current_battery_voltage > minimum_battery_voltage && number_of_low_battery_detections > 0)
  {
    number_of_low_battery_detections = number_of_low_battery_detections - 1;
  }
  
  
  if (current_voltage_index == number_stored_voltages - 1)
  {
    current_voltage_index = 0;
  }
  else
  {
    current_voltage_index = current_voltage_index+1;
  }
}

void checkBuckVoltageLevel()
{
  //retrieves the current voltage of the buck converter used to power the servos. Uses this and previous voltages to calculate the average output voltage of the buck, then publishes to pi
  const int buck_R1 = 10000;
  const int buck_R2 = 10000;
  
  int sensorValue = analogRead(A1);
  
  float current_voltage = sensorValue * (5.0 / 1023.0) * ((buck_R1+buck_R2)/buck_R2);
  servo_buck_voltage_array[current_voltage_index] = current_voltage;

  float average_voltage = 0;
  int numberOfVoltagesUsed = 0;

  for (int j = 0; j < number_stored_voltages; j++)
  {
    if (servo_buck_voltage_array[j] != 0.0)
    {
      average_voltage = average_voltage + servo_buck_voltage_array[j];
      numberOfVoltagesUsed = numberOfVoltagesUsed + 1;
    }
      
  } 

  float current_buck_voltage = average_voltage/numberOfVoltagesUsed;
  battery_voltage_msg.data = current_battery_voltage;
  battery_voltage_publisher.publish(&battery_voltage_msg);

  if (current_battery_voltage < min_buck_voltage)
  {
    number_of_low_battery_detections = number_of_low_battery_detections + 1;
    if (number_of_low_battery_detections > 30)
    {
      shutdown();
    }
  }
  else if (current_battery_voltage > minimum_battery_voltage && number_of_low_battery_detections > 0)
  {
    number_of_low_battery_detections = number_of_low_battery_detections - 1;
  }
  
  
  if (current_voltage_index == number_stored_voltages - 1)
  {
    current_voltage_index = 0;
  }
  else
  {
    current_voltage_index = current_voltage_index+1;
  }
}

void loop()
{
  //Runs every 100ms at the moment. Currently only checks the battery and buck voltage levels, but other functions can be added with more sensors.
  checkBatteryVoltageLevel();
  checkBuckVoltageLevel();
  arduino_nano.spinOnce();
  delay(100); //NOTE: ros-serial does not have an equivalent ros sleep function, so need to use native delay. Not a major issue for us.
}