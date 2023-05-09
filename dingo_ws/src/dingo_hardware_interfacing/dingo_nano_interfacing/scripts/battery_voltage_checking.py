import RPi.GPIO as GPIO

# Set the mode of the GPIO library
GPIO.setmode(GPIO.BCM)

# Set pin 5 as an input pin
GPIO.setup(5, GPIO.IN)

# Read the digital value from pin 5
value = GPIO.input(5)

# Print the value
print(value)

# Clean up the GPIO library
GPIO.cleanup()