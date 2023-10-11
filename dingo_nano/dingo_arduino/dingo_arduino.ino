#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <ElectricalMeasurements.h>
#include <avr/sleep.h>

ros::NodeHandle_<ArduinoHardware, 5, 5, 128, 256> arduino_nano_node; //(max) 5 publishers, 2 subscribers, 64 byte input buffer, 256 byte output buffer. Default is 25,25,512,512

//Declare global message variables for storing data to publish
std_msgs::Bool emergency_stop_msg;
dingo_peripheral_interfacing::ElectricalMeasurements electrical_measurement_msg;

//Declare publishers for data to be published on topics
ros::Publisher emergency_stop_status_publisher("emergency_stop_status", &emergency_stop_msg);
ros::Publisher electrical_measurement_publisher("electrical_measurements", &electrical_measurement_msg);

//Declare constants
const int publish_interval = 1; // publish voltage every 1 second

const int number_stored_battery_voltage_measurements = 30; // number of battery voltage values to average against
const int number_stored_buck_voltage_measurements = 5; // number of buck voltage values to average against

const float battery_R1 = 45.5; // resistor value between the pos battery terminal and nano pin
const float battery_R2 = 9.87; // resistor value between the neg battery terminal and nano pin
const float battery_minimum_voltage = 14.0;
const float battery_reference_voltage = 16.8;
const float battery_diode_drop = 0.2;
const float battery_fine_tuning = 1.067;

const float servobuck_R1 = 400.0; // resistor value between the pos buck terminal and nano pin
const float servobuck_R2 = 400.0; // resistor value between the neg buck terminal and nano pin
const float servobuck_minimum_voltage = 6.0;
const float servobuck_reference_voltage = 7.0;
const float servobuck_fine_tuning = 1.055;

// Declare global variables
float battery_buck_voltage_array[number_stored_battery_voltage_measurements] = {0.0};
float servobuck_voltage_array[number_stored_buck_voltage_measurements] = {0.0};

float battery_filtered_voltage = 0.0;
float servobuck_filtered_voltage = 0.0;

int buck_voltage_array_index = 0;
int number_of_low_battery_detections = 0;
int number_of_low_buck_voltage_detections = 0;
bool currently_estopped = 0;

void setup()
{
  // This function is required by arduino standards, and is run once on startup to initialise ros, publishers, subscribers and array initial values
  Serial.begin(57600);

  while (!Serial.available()) {
    delay(1000); // Wait 1 second before checking again
  }

  arduino_nano_node.getHardware()->setBaud(57600);
  // Initialise ros node, and advertise publishers
  arduino_nano_node.initNode();
  arduino_nano_node.advertise(emergency_stop_status_publisher);
  arduino_nano_node.advertise(electrical_measurement_publisher);

  // Multiple spins and delays to allow time for publishers to be properly setup before data is collected and transmitted
  arduino_nano_node.spinOnce(); delay(1000);
  arduino_nano_node.spinOnce(); delay(1000);
  arduino_nano_node.spinOnce(); delay(1000);
}

void shutdown()
{
  // Shuts the arduino down if the voltage level is too low

  // Log the shutdown in ROS logs
  arduino_nano_node.logwarn("BATTERY VOLTAGE TOO LOW: NANO POWERING DOWN");
  delay(1000);

  // Put arduino to sleep, which is as close to shutdown as possible
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  sleep_enable();
  sleep_cpu ();             
}

void checkBatteryVoltageLevel()
{
  // Retrieves the current voltage of the battery. Uses this and previous stored voltages to calculate the average voltage of the battery, then publishes to pi
  // This function also checks that the battery voltage is above the minimum, and will call the shutdown code if the voltage is too low
  
  // Get the voltage at the analogue port of the arduino
  int sensorValue = analogRead(A0);
  
  // Convert this voltage into the actual battery voltage and store into an array of voltage readings
  float pin_voltage = sensorValue * (5.0 / 1023.0);
  float current_voltage = pin_voltage * ((battery_R1+battery_R2)/battery_R2) + battery_diode_drop;

  // Fine tuning adjustment
  current_voltage = current_voltage * battery_fine_tuning;

  float battery_average_voltage = calculateAverageVoltage(
    current_voltage, battery_buck_voltage_array, number_stored_battery_voltage_measurements, battery_filtered_voltage, battery_reference_voltage);

  // Serial.println(String(pin_voltage));
  // Serial.println(String(battery_average_voltage));

  // Write the current average voltage to the publisher message
  electrical_measurement_msg.battery_voltage_level = battery_average_voltage;

  // Check whether the voltage is below the minimum. If so, increment the number of low battery detections. If too many have occured, order shutdown of nano
  if (battery_average_voltage < battery_minimum_voltage)
  {
    if (number_of_low_battery_detections > 30)
    {
      shutdown(); // TODO: ROS topic to shutdown pi as well
    }
    else
    {
      number_of_low_battery_detections++;
    }
    
  } // Decrement number of low battery detections if it was above zero and the battery voltage is above the minimum acceptable
  else if (battery_average_voltage > battery_minimum_voltage && number_of_low_battery_detections > 0)
  {
    number_of_low_battery_detections = number_of_low_battery_detections - 1;
  }
}

void checkBuckVoltageLevel()
{
  // Retrieves the current voltage of the buck converter output. Uses this and previous voltages to calculate the average voltage of the buck converter output, then publishes to pi
  // This function also checks whether the buck converter has dropped to zero. If it has for long enough, the e-stop must have been pressed to shut down power to the buck and servos, so publish this
  // Also checks when the voltage increases again, indicating that the e-stop has been released, and publishes this as well

  // Get the voltage at the analogue port of the arduino
  int sensorValue = analogRead(A1);

  // Convert this voltage into the actual buck converter output voltage and store into an array of voltage readings
  float pin_voltage = sensorValue * (5.0 / 1023.0);
  float current_voltage = pin_voltage * ((servobuck_R1+servobuck_R2)/servobuck_R2);

  // Fine tuning adjustment
  current_voltage = current_voltage * servobuck_fine_tuning;

  float servobuck_average_voltage = calculateAverageVoltage(
    current_voltage, servobuck_voltage_array, number_stored_buck_voltage_measurements, servobuck_filtered_voltage, servobuck_reference_voltage);

  // Serial.println(String(pin_voltage));
  // Serial.println(String(servobuck_average_voltage));

  // Write the current average voltage to the publisher message
  electrical_measurement_msg.servo_buck_voltage_level = servobuck_average_voltage;

  // Check whether the buck converter voltage has dropped to 0 (or near 0). If it has, send message that the e-stop has been pressed
  if (servobuck_average_voltage < servobuck_minimum_voltage)
  {
    if (number_of_low_buck_voltage_detections < 3)
    {
      number_of_low_buck_voltage_detections = number_of_low_buck_voltage_detections + 1;
    }
    else if (currently_estopped == 0)
    {
      currently_estopped = 1;
      emergency_stop_msg.data = 1;
      emergency_stop_status_publisher.publish(&emergency_stop_msg);

    }
  } // If buck converter voltage has recently increased from 0, send message that e-stop has been released
  else if (servobuck_average_voltage > servobuck_minimum_voltage)
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

  // Increment the index for where electrical measurements are kept inside respective array
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
  // This function is required by arduino and runs constantly in a loop while it is powered
  // It calls functions to measure sensor data on every loop, publishing this data to topics. Also spins topics to clear buffers

  static unsigned long lastPublishMillis = 0;  // last time the message was published
  unsigned long currentMillis = millis();  // current time
    
  // Check voltage of battery and buck converter output
  checkBatteryVoltageLevel();
  checkBuckVoltageLevel();

  
  // Publish electrical measurements to the electrical measurement topic
  if (currentMillis - lastPublishMillis >= publish_interval * 1000) {
    electrical_measurement_publisher.publish(&electrical_measurement_msg);
    lastPublishMillis = currentMillis;
  }

  // Spin nodes
  arduino_nano_node.spinOnce();

  // Runs every 100ms at the moment.
  delay(100); //NOTE: ros-serial does not have an equivalent ros sleep function, so need to use native delay. Not a major issue for us.
  
}

float calculateAverageVoltage(float last_voltage, float voltage_array[], int voltage_array_size, float& filtered_voltage, float reference_voltage) {
    float sum_of_voltages = 0.0;

    // Shift voltages in array
    for(int i = 0; i < voltage_array_size-1; i++) {
        voltage_array[i] = voltage_array[i + 1];
    }
    // Assign the last voltage reading to the last element
    voltage_array[voltage_array_size-1] = last_voltage;

    // Calculate the current average of all the voltages in the array
    for (int i = 0; i < voltage_array_size; i++) {
        sum_of_voltages += voltage_array[i];
    }
    
    float average_voltage = (sum_of_voltages / voltage_array_size);
    
    // The filtered voltage should just be set to reference voltage if averaging array is not fully populated
    // And battery voltage should only update if current average is lower than the previous average.
    if (reference_voltage == servobuck_reference_voltage)
    {
      filtered_voltage = average_voltage;   
    } else {
      if (find_min(voltage_array, voltage_array_size) == 0) {
          filtered_voltage = reference_voltage;
      } else if (average_voltage < filtered_voltage) {
          filtered_voltage = average_voltage;
      } 
    } 

    return filtered_voltage;
}

int find_min(float arr[], int n) {
    int min_val = arr[0];
    for(int i = 1; i < n; i++) {
        if(arr[i] < min_val) {
            min_val = arr[i];
        }
    }
    return min_val;
}

int find_max(float arr[], int n) {
    int max_val = arr[0];
    for(int i = 1; i < n; i++) {
        if(arr[i] > max_val) {
            max_val = arr[i];
        }
    }
    return max_val;
}