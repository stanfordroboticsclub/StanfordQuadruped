#!/usr/bin/env python3
import rospy
import spidev as SPI
import LCD_1inch47
from PIL import Image, ImageDraw, ImageFont
import logging
import os
import rospkg
import socket
import time
from std_msgs.msg import Float64
from dingo_peripheral_interfacing.msg import ElectricalMeasurements

rospack = rospkg.RosPack()
rospack.get_path('dingo_peripheral_interfacing') + "/lib/emptybatterystatus_white.png"

class DingoDisplayNode:
    def __init__(self):
        rospy.init_node('dingo_display_node')
        self.rate = rospy.Rate(50)  # 50 Hz
        # Raspberry Pi pin configuration:
        self.RST = 27
        self.DC = 25
        self.BL = 18
        self.bus = 0
        self.device = 0
        self.ssid = ''
        self.ipAddress = ''
        self.disp = LCD_1inch47.LCD_1inch47()
        # Initialize library.
        self.disp.Init()
        # Clear display.
        self.disp.clear()

        self.battery_voltage_subscriber = rospy.Subscriber("/electrical_measurements", ElectricalMeasurements, self.update_battery_percentage)

        self.battery_percentage = 0.7  # Number between 0 and 1

    def update_battery_percentage(self, message):

        # max_voltage and min_voltage for 4s lipo battery
        max_voltage = 16.8
        min_voltage = 14.0

        # Ensure the received voltage is within expected bounds
        battery_voltage_level = max(min(message.battery_voltage_level, max_voltage), min_voltage)

        # Convert to percentage
        self.battery_percentage = (battery_voltage_level - min_voltage) / (max_voltage - min_voltage)

        # rospy.loginfo("Battery voltage level: {}".format(message.battery_voltage_level))
        # rospy.loginfo("Updated battery percentage: {:.2f}%".format(self.battery_percentage * 100))

    def run(self):
        try:
            # display with hardware SPI:
            #rospy.loginfo("Initializing display...")

            # Create blank image for drawing.
            image1 = Image.new("RGB", (self.disp.height, self.disp.width), "black")
            draw = ImageDraw.Draw(image1)

            #rospy.loginfo("Importing fonts...")
            Font1 = ImageFont.truetype("/usr/share/fonts/truetype/Font02.ttf", 25)
            Font1_small = ImageFont.truetype("/usr/share/fonts/truetype/Font02.ttf", 20)
            Font1_large = ImageFont.truetype("/usr/share/fonts/truetype/Font02.ttf", 60)
            Font2 = ImageFont.truetype("/usr/share/fonts/truetype/Font01.ttf", 35)
            Font3 = ImageFont.truetype("/usr/share/fonts/truetype/Font02.ttf", 120)

            # draw.text((60, -15), 'Dingo', fill="RED", font=Font3)
            draw.text((20, 110), 'SSID: '+ self.ssid, fill="WHITE", font=Font1)
            draw.text((20, 135), 'IP: ' + self.ipAddress, fill="WHITE", font=Font1)
            current_time = time.strftime("%I:%M:%S%p")
            draw.text((220, 0), current_time, fill="WHITE", font=Font1_small)

            ## Battery indication bar
            black= Image.new("RGB", (320, 172), "black")

            batt_status = Image.open(rospack.get_path('dingo_peripheral_interfacing') + "/lib/emptybatterystatus_white.png")

            # batt_status = batt_status.rotate(180)
            batt_draw = ImageDraw.Draw(batt_status)

            if self.battery_percentage <=0.20:
                batt_fill = "RED"
            elif 0.20 < self.battery_percentage <=0.60:
                batt_fill = "#d49b00" #yellow
            else:
                batt_fill = "#09ab00" #green

            batt_draw.rounded_rectangle([(42,92),(42+(153*self.battery_percentage) ,170)],8,fill = batt_fill)
            batt_draw.text((68, 95),str(int(self.battery_percentage*100))+"%", fill = "WHITE",font = Font1_large)
            batt_scale_factor=0.8
            resized_batt_status = batt_status.resize((int(batt_status.size[0]*batt_scale_factor),int(batt_status.size[1]*batt_scale_factor)))
            image1.paste(resized_batt_status,(62, -40),resized_batt_status.convert('RGBA'))


            #rospy.loginfo("Drawing...")
            image1 = image1.rotate(0)
            image1 = image1.transpose(Image.ROTATE_270)
            self.disp.ShowImage(image1)

            ## SSID and IP are calcultated after displaying so that they can run on the second loop
            #rospy.loginfo("Connecting to Wi-Fi...")
            try:
                self.ssid = os.popen("iwgetid -r").read().strip()
            except rospy.ROSInterruptException as e:
                rospy.logerr(str(e))
                self.ssid = "N/A"

            #rospy.loginfo("SSID: " + str(self.ssid))
            #rospy.loginfo("Getting IP address...")
            try:
                #s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                #s.connect(('google.com', 0))
                #self.ipAddress = s.getsockname()[0]
                #s.close()
                hostname=socket.gethostname()   
                self.ipAddress=socket.gethostbyname(hostname) 
            except rospy.ROSInterruptException as e:
                rospy.logerr(str(e))
                self.ipAddress = '-:-:-:-'
            #rospy.loginfo("IP: {self.ipAddress}")

        except rospy.ROSInterruptException as e:
            rospy.logerr(str(e))
            

    def loop(self):
        while not rospy.is_shutdown():
            self.run()
            self.rate.sleep()
        else:
            rospy.loginfo("Quitting...")
            self.disp.clear()
            self.disp.module_exit()
            print("finally 1")


if __name__ == '__main__':
    node = DingoDisplayNode()
    rospy.loginfo("Display node started, ouputting to display")
    node.loop()

