#!/usr/bin/env python
import rospy
import spidev as SPI
import LCD_1inch47
from PIL import Image, ImageDraw, ImageFont
import logging
import os
import socket
import time

class DingoDisplayNode:
    def __init__(self):
        rospy.init_node('dingo_display_node')
        self.rate = rospy.Rate(10)  # 10 Hz
        self.logger = logging.getLogger('dingo_display_node')
        self.logger.setLevel(logging.DEBUG)
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

    def run(self):
        try:
            # display with hardware SPI:
            self.logger.info("Initializing display...")

            # Create blank image for drawing.
            image1 = Image.new("RGB", (self.disp.height, self.disp.width), "black")
            draw = ImageDraw.Draw(image1)

            self.logger.info("Importing fonts...")
            Font1 = ImageFont.truetype("/usr/share/fonts/truetype/Font02.ttf", 25)
            Font1_small = ImageFont.truetype("/usr/share/fonts/truetype/Font02.ttf", 20)
            Font2 = ImageFont.truetype("/usr/share/fonts/truetype/Font01.ttf", 35)
            Font3 = ImageFont.truetype("/usr/share/fonts/truetype/Font02.ttf", 120)

            draw.text((60, -15), 'Dingo', fill="RED", font=Font3)
            draw.text((20, 110), f'SSID: {self.ssid}', fill="WHITE", font=Font1)
            draw.text((20, 135), f'IP: {self.ipAddress}', fill="WHITE", font=Font1)
            current_time = time.strftime("%I:%M:%S%p")
            draw.text((220, 0), current_time, fill="WHITE", font=Font1_small)

            self.logger.info("Drawing...")
            image1 = image1.rotate(0)
            image1 = image1.transpose(Image.ROTATE_270)
            self.disp.ShowImage(image1)

            ## SSID and IP are calcultated after displaying so that they can run on the second loop
            self.logger.info("Connecting to Wi-Fi...")
            try:
                self.ssid = os.popen("iwgetid -r").read().strip()
            except rospy.ROSInterruptException as e:
                self.logger.error(str(e))
                self.ssid = "N/A"

            self.logger.info(f"SSID: {self.ssid}")
            self.logger.info("Getting IP address...")
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(('google.com', 0))
                self.ipAddress = s.getsockname()[0]
            except rospy.ROSInterruptException as e:
                self.logger.error(str(e))
                self.ipAddress = '-:-:-:-'
            s.close()
            self.logger.info(f"IP: {self.ipAddress}")

        except rospy.ROSInterruptException as e:
            self.logger.error(str(e))
            

    def loop(self):
        while not rospy.is_shutdown():
            self.run()
            self.rate.sleep()
        else:
            self.logger.info("Quitting...")
            self.disp.clear()
            self.disp.module_exit()
            print("finally 1")


if __name__ == '__main__':
    node = DingoDisplayNode()
    rospy.loginfo("Display node started, ouputting to display")
    node.loop()