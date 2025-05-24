# This program demonstrates how to control two DC motors using the MDD10 HAT in Lock-Antiphase mode.
# It uses an ultrasonic sensor to maintain a specified distance from a wall.
# The robot moves forward or backward while adjusting its speed based on proportional-integral control (PI).

import RPi.GPIO as GPIO          # Import GPIO library for Raspberry Pi
from time import sleep, time     # Import sleep for delays and time for timestamps
from Components import Robot
GPIO.setmode(GPIO.BCM)           # Use Broadcom (BCM) pin numbering
GPIO.setwarnings(False)          # Suppress GPIO warnings



# Main setup

robot = Robot()


# === Main Test Sequence ===


try:
    while True:
        mariusz = 2

except KeyboardInterrupt:
    GPIO.cleanup()
    print("Program zatrzymany.")
