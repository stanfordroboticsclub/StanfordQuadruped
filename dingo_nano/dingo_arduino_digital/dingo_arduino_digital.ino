#include <avr/sleep.h>

//Declare global variables
const int number_stored_battery_voltage_measurements = 40; //number of electrical measurements (e.g. voltage values) to keep at any one time
const int number_stored_buck_voltage_measurements = 3;
float battery_voltage_array[number_stored_battery_voltage_measurements];
float servo_buck_voltage_array[number_stored_buck_voltage_measurements];
int battery_voltage_array_index = 0; //variable for the index at which the current measurement is stored in the corresponding array of measurements
int buck_voltage_array_index = 0;
int number_of_low_battery_detections = 0;
int number_of_low_buck_voltage_detections = 0;
bool currently_estopped = 0;

const int eStopStatusPin = 2;
const int battPercentPin1 = 3;
const int battPercentPin2 = 4;
const int battPercentPin3 = 5;

float voltage_ceiling = 1.0;

void setup()
{
  delay(60000);
  Serial.begin(9600);
  pinMode(eStopStatusPin, OUTPUT);
  pinMode(battPercentPin1, OUTPUT);
  pinMode(battPercentPin2, OUTPUT);
  pinMode(battPercentPin3, OUTPUT);

  digitalWrite(eStopStatusPin, 0);
  digitalWrite(battPercentPin1, 0);
  digitalWrite(battPercentPin2, 0);
  digitalWrite(battPercentPin3, 0);


  //go through arrays of measurements and set all values to initially be zero
  for (int i = 0; i < number_stored_battery_voltage_measurements; i++)
  {
    battery_voltage_array[i] = 0.0;
    
  }

  for (int i = 0; i < number_stored_buck_voltage_measurements; i++)
  {
    servo_buck_voltage_array[i] = 0.0;
    
  }
  
}

void shutdown()
{
  //shuts the arduino down if the voltage level is too low

  delay(25000);

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
  const float battery_R1 = 26.1; //^3, resistance value of the resistor between the tve battery terminal and the pin
  const float battery_R2 = 9.96; //^3, resistance value of the resistor between the -ve battery terminal and the pin
  const int minimum_battery_voltage = 15;
  const float diode_drop = 0.22;
  const float maximum_battery_voltage = 16.8;
  
  //Get the voltage at the analogue port of the arduino
  int sensorValue = analogRead(A0);
  
  //convert this voltage into the actual battery voltage and store into an array of voltage readings
  float pin_voltage = sensorValue * (5.0 / 1023.0);
  float current_voltage = pin_voltage * ((battery_R1+battery_R2)/battery_R2) + diode_drop;

  current_voltage = current_voltage;

  Serial.print(current_voltage);
  Serial.print("\n");

  battery_voltage_array[battery_voltage_array_index] = current_voltage;

  //using the array of the most recent voltage readings, calculate the current average voltage of the battery
  float sum_of_voltages = 0;
  int numberOfVoltagesUsed = 0;
  for (int j = 0; j < number_stored_battery_voltage_measurements; j++)
  {
    if (battery_voltage_array[j] > 2.0)
    {
      sum_of_voltages = sum_of_voltages + battery_voltage_array[j];
      numberOfVoltagesUsed = numberOfVoltagesUsed + 1;
    }
      
  } 
  //note: if the sum of voltages is zero, set current_battery_voltage to zero to avoid NaN from 0/x
  float current_battery_voltage;
  if (sum_of_voltages != 0.0)
  {
    current_battery_voltage = sum_of_voltages/numberOfVoltagesUsed;
  }
  else
  {
    current_battery_voltage = 0.0;
  }

  float amount_batt_volt_above_min = current_battery_voltage - minimum_battery_voltage;
  float max_amount_batt_volt_above_min = maximum_battery_voltage - minimum_battery_voltage;

  float battery_percentage = amount_batt_volt_above_min/max_amount_batt_volt_above_min;

  Serial.print(battery_percentage);
  Serial.print("\n");

  //if (battery_percentage > voltage_ceiling + 0.0625)
  //{
  //  voltage_ceiling+=0.125;
  //}

  if (battery_percentage <= 0.0)
  {
    battery_percentage = 0.0;
  }

  



  int bit1 = 0;
  int bit2 = 0;
  int bit3 = 0;
  if (battery_percentage <= 0.0){
    bit1 = 0;
    bit2 = 0;
    bit3 = 0;
    Serial.print("\nhere1");
    voltage_ceiling = 0.0;
  } else if (battery_percentage <= 0.125) {
    bit1 = 1;
    bit2 = 0;
    bit3 = 0;
    Serial.print("\nhere2");
    voltage_ceiling = 0.125;
  } else if (battery_percentage <= 0.25) {
    bit1 = 0;
    bit2 = 1;
    bit3 = 0;
    Serial.print("\nhere3");
    voltage_ceiling = 0.25;
  } else if (battery_percentage <= 0.375) {
    bit1 = 1;
    bit2 = 1;
    bit3 = 0;
    Serial.print("\nhere4");
    voltage_ceiling = 0.375;
  } else if (battery_percentage <= 0.5) {
    bit1 = 0;
    bit2 = 0;
    bit3 = 1;
    Serial.print("\nhere5");
    voltage_ceiling = 0.5;
  } else if (battery_percentage <= 0.625) {
    bit1 = 1;
    bit2 = 0;
    bit3 = 1;
    Serial.print("\nhere6");
    voltage_ceiling = 0.625;
  } else if (battery_percentage <= 0.75) {
    bit1 = 0;
    bit2 = 1;
    bit3 = 1;
    Serial.print("\nhere7");
    voltage_ceiling = 0.75;
  } else {
    bit1 = 1;
    bit2 = 1;
    bit3 = 1;
    Serial.print("\nhere8");
  }
  // Write the battery level as a percentage to the LED pins
  digitalWrite(battPercentPin1, bit1);   // write HIGH or LOW based on the percentage
  digitalWrite(battPercentPin2, bit2);
  digitalWrite(battPercentPin3, bit3);

  //Check whether the voltage is below the minimum. If so, increment the number of low battery detections. If too many have occured, order shutdown of nano
  if (current_battery_voltage < minimum_battery_voltage)
  {
    if (number_of_low_battery_detections > 30)
    {
      //shutdown();
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

  //increment the index for where electrical measurements are kept inside respective array
  if (battery_voltage_array_index == number_stored_battery_voltage_measurements - 1)
  {
    battery_voltage_array_index = 0;
  }
  else
  {
    battery_voltage_array_index++;
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
  const int minimum_buck_voltage = 6;
  
  //Get the voltage at the analogue port of the arduino
  int sensorValue = analogRead(A1);

  //convert this voltage into the actual buck converter output voltage and store into an array of voltage readings
  float current_voltage = sensorValue * (5.0 / 1023.0) * ((buck_R1+buck_R2)/buck_R2);

  current_voltage = current_voltage;

  //Arduino will read a NaN when the voltage is zero, so account for this

  servo_buck_voltage_array[buck_voltage_array_index] = current_voltage;

  //using the array of the most recent voltage readings, calculate the current average voltage of the buck converter output
  float sum_of_voltages = 0;
  int numberOfVoltagesUsed = 0;
  for (int j = 0; j < number_stored_buck_voltage_measurements; j++)
  {
    if (servo_buck_voltage_array[j] > 2.0)
    {
      sum_of_voltages = sum_of_voltages + servo_buck_voltage_array[j];
      numberOfVoltagesUsed = numberOfVoltagesUsed + 1;
    }
      
  } 
  //note: if the sum of voltages is zero, set current_buck_voltage to zero to avoid NaN from 0/x
  float current_buck_voltage;
  if (sum_of_voltages != 0.0)
  {
    current_buck_voltage = sum_of_voltages/numberOfVoltagesUsed;
  }
  else
  {
    current_buck_voltage = 0.0;
  }

  //check whether the buck converter voltage has dropped to 0 (or near 0). If it has, send message that the e-stop has been pressed


  if (current_buck_voltage < minimum_buck_voltage)
  {
    if (number_of_low_buck_voltage_detections < 3)
    {
      number_of_low_buck_voltage_detections = number_of_low_buck_voltage_detections + 1;
    }
    else if (currently_estopped == 0)
    {
      currently_estopped = 1;
      digitalWrite(eStopStatusPin, 1);

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
      digitalWrite(eStopStatusPin, 0); 
    }
  }

  //increment the index for where electrical measurements are kept inside respective array
  if (buck_voltage_array_index == number_stored_buck_voltage_measurements - 1)
  {
    buck_voltage_array_index = 0;
  }
  else
  {
    buck_voltage_array_index++;
  }
}

void loop()
{
  //This function is required by arduino and runs constantly in a loop while it is powered
  //it calls functions to measure sensor data on every loop, publishing this data to topics. Also spins topics to clear buffers
  
  //check voltage of battery and buck converter output
  checkBatteryVoltageLevel();
  checkBuckVoltageLevel();

  //Runs every 100ms at the moment.
  delay(100); //NOTE: ros-serial does not have an equivalent ros sleep function, so need to use native delay. Not a major issue for us.
  
}