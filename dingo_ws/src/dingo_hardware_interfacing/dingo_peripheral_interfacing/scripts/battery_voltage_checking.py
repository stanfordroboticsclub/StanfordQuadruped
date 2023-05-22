import RPi.GPIO as GPIO
import sys, rospy, signal, subprocess, time
from std_msgs.msg import Float64, Bool

def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)

def shutdown():
    GPIO.cleanup()
    rospy.logwarn("BATTERY VOLTAGE TOO LOW. COMMENCING SHUTDOWN PROCESS")
    time.sleep(5)
    subprocess.run(["sudo", "shutdown", "-h", "now"])

def main():
    # Set the mode of the GPIO library
    rospy.init_node("battery_monitor") 
    message_rate = 50
    rate = rospy.Rate(message_rate)

    signal.signal(signal.SIGINT, signal_handler)

    GPIO.setmode(GPIO.BCM)

    estop_pin_number = 5
    battery_pin1_number = 6
    battery_pin2_number = 13
    battery_pin3_number = 19

    # Set pin 5 as an input pin
    GPIO.setup(estop_pin_number, GPIO.IN)
    GPIO.setup(battery_pin1_number, GPIO.IN)
    GPIO.setup(battery_pin2_number, GPIO.IN)
    GPIO.setup(battery_pin3_number, GPIO.IN)

    battery_percentage_publisher = rospy.Publisher("/battery_percentage", Float64, queue_size = 10)
    estop_publisher = rospy.Publisher("/emergency_stop_status", Bool, queue_size = 10)
    current_estop_bit = 0

    number_of_low_battery_detections = 0

    estop_bit = GPIO.input(estop_pin_number)
    battery_bit1 = GPIO.input(battery_pin1_number)
    battery_bit2 = GPIO.input(battery_pin2_number)
    battery_bit3 = GPIO.input(battery_pin3_number)

    #Grab initial value and publish that immediately
    if estop_bit == 0:
        estop_publisher.publish(0)
    elif estop_bit == 1:
        estop_publisher.publish(1)
        current_estop_bit = 1
    
    while not rospy.is_shutdown(): 
        # Read the digital values from the pins
        estop_bit = GPIO.input(estop_pin_number)
        battery_bit1 = GPIO.input(battery_pin1_number)
        battery_bit2 = GPIO.input(battery_pin2_number)
        battery_bit3 = GPIO.input(battery_pin3_number)
        print("estop: ", battery_bit1)
        print("bit1: ", battery_bit1)
        print("bit2: ", battery_bit2)
        print("bit3: ", battery_bit3)

        battery_bits = [battery_bit1, battery_bit2, battery_bit3]

        if estop_bit == 1 and current_estop_bit == 0:
            current_estop_bit = 1
            estop_publisher.publish(1)

        if estop_bit == 0 and current_estop_bit == 1:
            current_estop_bit = 0
            estop_publisher.publish(0)

        # Convert the bits to a decimal number
        num = int("".join([str(b) for b in battery_bits]), 2)

        value = 0.0

        # Check which scenario has occurred
        if num == 0:
            value = 0.0
        elif num == 1:
            value = 0.125
        elif num == 2:
            value = 0.25
        elif num == 3:
            value = 0.375
        elif num == 4:
            value = 0.5
        elif num == 5:
            value = 0.625
        elif num == 6:
            value = 0.75
        elif num == 7:
            value = 1

        battery_percentage_publisher.publish(value)

        if value == 0.0:
            number_of_low_battery_detections = number_of_low_battery_detections+1
            if (number_of_low_battery_detections > 30):
                #shutdown()
                print("Would shut down if activated")
        else:
            if (number_of_low_battery_detections > 0):
                number_of_low_battery_detections = number_of_low_battery_detections-1


        rate.sleep()

main()