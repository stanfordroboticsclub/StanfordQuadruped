#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <dingo_nano_interfacing/ElectricalMeasurements.h>
#include <avr/sleep.h>

ros::NodeHandle_<ArduinoHardware, 5, 5, 256, 256> arduino_nano_node; //(max) 5 publishers, 2 subscribers, 64 byte input buffer, 256 byte output buffer. Default is 25,25,512,512

//Declare global message variables for storing data to publish
std_msgs::Bool emergency_stop_msg;
dingo_nano_interfacing::ElectricalMeasurements electrical_measurement_msg;

//Declare publishers for data to be published on topics
ros::Publisher emergency_stop_status_publisher("emergency_stop_status", &emergency_stop_msg);
ros::Publisher electrical_measurement_publisher("electrical_measurements", &electrical_measurement_msg);

//Declare subscribers for data to be received when published to a topic


//Declare global variables
const int number_stored_electrical_measurements = 10; //number of electrical measurements (e.g. voltage values) to keep at any one time
float battery_voltage_array[number_stored_electrical_measurements];
float servo_buck_voltage_array[number_stored_electrical_measurements];
int electrical_measurement_array_index = 0; //variable for the index at which the current measurement is stored in the corresponding array of measurements
int number_of_low_battery_detections = 0;
int number_of_low_buck_voltage_detections = 0;
bool currently_estopped = 0;

void setup()
{
  //This function is required by arduino standards, and is run once on startup to initialise ros, publishers, subscribers and array initial values

  //initialise ros node, and advertise publishers
  arduino_nano_node.initNode();
  arduino_nano_node.advertise(emergency_stop_status_publisher);
  arduino_nano_node.advertise(electrical_measurement_publisher);

  //multiple spins and delays to allow time for publishers to be properly setup before data is collected and transmitted
  arduino_nano_node.spinOnce();
  delay(1000);
  arduino_nano_node.spinOnce();
  delay(1000);
  arduino_nano_node.spinOnce();
  delay(1000);

  //go through arrays of measurements and set all values to initially be zero
  for (int i = 0; i < number_stored_electrical_measurements; i++)
  {
    battery_voltage_array[i] = 0.0;
    servo_buck_voltage_array[i] = 0.0;
  }
}

void shutdown()
{
  //shuts the arduino down if the voltage level is too low

  //Log the shutdown in ROS logs
  arduino_nano_node.logwarn("BATTERY VOLTAGE TOO LOW: NANO POWERING DOWN");
  delay(1000);

  //Put arduino to sleep, which is as close to shutdown as possible
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  sleep_enable();
  sleep_cpu ();             
}

void checkBatteryVoltageLevel()
{
  //retrieves the current voltage of the battery. Uses this and previous stored voltages to calculate the average voltage of the battery, then publishes to pi
  //This function also checks that the battery voltage is above the minimum, and will call the shutdown code if the voltage is too low

  //Declare constants used by the function
  const int battery_R1 = 27; //^3, resistance value of the resistor between the tve battery terminal and the pin
  const int battery_R2 = 10; //^3, resistance value of the resistor between the -ve battery terminal and the pin
  const int minimum_battery_voltage = 15;
  
  //Get the voltage at the analogue port of the arduino
  int sensorValue = analogRead(A0);
  
  //convert this voltage into the actual battery voltage and store into an array of voltage readings
  float pin_voltage = sensorValue * (5.0 / 1023.0);
  float current_voltage = pin_voltage * ((battery_R1+battery_R2)/battery_R2) + 0.7;
  battery_voltage_array[electrical_measurement_array_index] = current_voltage;

  //using the array of the most recent voltage readings, calculate the current average voltage of the battery
  float average_voltage = 0;
  int numberOfVoltagesUsed = 0;
  for (int j = 0; j < number_stored_electrical_measurements; j++)
  {
    if (battery_voltage_array[j] > 2.0)
    {
      average_voltage = average_voltage + battery_voltage_array[j];
      numberOfVoltagesUsed = numberOfVoltagesUsed + 1;
    }
      
  } 
  float current_battery_voltage = average_voltage/numberOfVoltagesUsed;

  //write the current average voltage to the publisher message
  electrical_measurement_msg.battery_voltage_level = current_battery_voltage;

  //Check whether the voltage is below the minimum. If so, increment the number of low battery detections. If too many have occured, order shutdown of nano
  if (current_battery_voltage < minimum_battery_voltage)
  {
    if (number_of_low_battery_detections > 30)
    {
      shutdown();
    }
    else
    {
      number_of_low_battery_detections++;
    }
    
  } //Decrement number of low battery detections if it was above zero and the battery voltage is above the minimum acceptable
  else if (current_battery_voltage > minimum_battery_voltage && number_of_low_battery_detections > 0)
  {
    number_of_low_battery_detections = number_of_low_battery_detections - 1;
  }
}

void checkBuckVoltageLevel()
{
  //retrieves the current voltage of the buck converter output. Uses this and previous voltages to calculate the average voltage of the buck converter output, then publishes to pi
  //This function also checks whether the buck converter has dropped to zero. If it has for long enough, the e-stop must have been pressed to shut down power to the buck and servos, so publish this
  //Also checks when the voltage increases again, indicating that the e-stop has been released, and publishes this as well

  //Declare constants used by the function
  const int buck_R1 = 10; //^3
  const int buck_R2 = 10; //^3
  const int minimum_buck_voltage = 2;
  
  //Get the voltage at the analogue port of the arduino
  int sensorValue = analogRead(A1);
  
  //convert this voltage into the actual buck converter output voltage and store into an array of voltage readings
  float current_voltage = sensorValue * (5.0 / 1023.0) * ((buck_R1+buck_R2)/buck_R2);
  servo_buck_voltage_array[electrical_measurement_array_index] = current_voltage;

  //using the array of the most recent voltage readings, calculate the current average voltage of the buck converter output
  float average_voltage = 0;
  int numberOfVoltagesUsed = 0;
  for (int j = 0; j < number_stored_electrical_measurements; j++)
  {
    if (servo_buck_voltage_array[j] > 2.0)
    {
      average_voltage = average_voltage + servo_buck_voltage_array[j];
      numberOfVoltagesUsed = numberOfVoltagesUsed + 1;
    }
      
  } 
  float current_buck_voltage = average_voltage/numberOfVoltagesUsed;

  //write the current average voltage to the publisher message
  electrical_measurement_msg.servo_buck_voltage_level = current_buck_voltage;

  //check whether the buck converter voltage has dropped to 0 (or near 0). If it has, send message that the e-stop has been pressed
  if (current_buck_voltage < minimum_buck_voltage)
  {
    if (number_of_low_buck_voltage_detections < 5)
    {
      number_of_low_buck_voltage_detections = number_of_low_buck_voltage_detections + 1;
    }
    else if (currently_estopped == 0)
    {
      currently_estopped = 1;
      emergency_stop_msg.data = 1;
      emergency_stop_status_publisher.publish(&emergency_stop_msg);

    }
  } //if buck converter voltage has recently increased from 0, send message that e-stop has been released
  else if (current_buck_voltage > minimum_buck_voltage)
  {
    if (number_of_low_buck_voltage_detections > 0)
    {
      number_of_low_buck_voltage_detections = number_of_low_buck_voltage_detections - 1;
    }
    else if (currently_estopped == 1)
    {
      currently_estopped = 0;
      emergency_stop_msg.data = 0;
      emergency_stop_status_publisher.publish(&emergency_stop_msg);
    }
  }
}

void loop()
{
  //This function is required by arduino and runs constantly in a loop while it is powered
  //it calls functions to measure sensor data on every loop, publishing this data to topics. Also spins topics to clear buffers
  
  //check voltage of battery and buck converter output
  checkBatteryVoltageLevel();
  checkBuckVoltageLevel();

  //publish electrical measurements to the electrical measurement topic
  electrical_measurement_publisher.publish(&electrical_measurement_msg);

  //increment the index for where electrical measurements are kept inside respective array
  if (electrical_measurement_array_index == number_stored_electrical_measurements - 1)
  {
    electrical_measurement_array_index = 0;
  }
  else
  {
    electrical_measurement_array_index = electrical_measurement_array_index+1;
  }

  //spin nodes
  arduino_nano_node.spinOnce();

  //Runs every 100ms at the moment.
  delay(100); //NOTE: ros-serial does not have an equivalent ros sleep function, so need to use native delay. Not a major issue for us.
  
}