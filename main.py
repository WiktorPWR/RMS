# This program demonstrates how to control two DC motors using the MDD10 HAT in Lock-Antiphase mode.
# It uses an ultrasonic sensor to maintain a specified distance from a wall.
# The robot moves forward or backward while adjusting its speed based on proportional-integral control (PI).

import RPi.GPIO as GPIO          # Import GPIO library for Raspberry Pi
from time import sleep, time     # Import sleep for delays and time for timestamps
from Components.Robot import Robot

GPIO.setmode(GPIO.BCM)           # Use Broadcom (BCM) pin numbering
GPIO.setwarnings(False)          # Suppress GPIO warnings

class MainController:
    def __init__(self, status_log):
        self.status_log = status_log

        self.distance_between_floor_endstops = 100 # Distance between endstops in cm (Check and adjust)!!!
        self.robot = Robot(self.distance_between_floor_endstops, status_log)
        
    def log(self, message, type = "info"):
        self.status_log.append({"source": "[MainController]", "message": message, "type": type})

    def malowanie(self, x, z):
        self.log("Rozpoczynam malowanie", "info")
        self.robot.platform.move_z_axis(z)
        self.robot.move_forward(x, paint_or_not=True)
        self.log("Malowanie zakończone", "success")

    def kalibracja(self):
        self.log("Rozpoczynam kalibracje", "info")
        #------------------------------------------------------------------
        #Trzeba dorobić logike do kalibracji bo nie widze żeby była gdzieś
        #------------------------------------------------------------------
        sleep(2)
        self.log("Kalibracja zakończona", "success")

status_log = []
main_controller = MainController(status_log)
