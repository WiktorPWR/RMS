# This program demonstrates how to control two DC motors using the MDD10 HAT in Lock-Antiphase mode.
# It uses an ultrasonic sensor to maintain a specified distance from a wall.
# The robot moves forward or backward while adjusting its speed based on proportional-integral control (PI).

import RPi.GPIO as GPIO          # Import GPIO library for Raspberry Pi
from time import sleep, time     # Import sleep for delays and time for timestamps
from Components.Robot import Robot
from datetime import datetime

GPIO.setmode(GPIO.BCM)           # Use Broadcom (BCM) pin numbering
GPIO.setwarnings(False)          # Suppress GPIO warnings

class MainController:
    def __init__(self, status_log):
        self.status_log = status_log

        self.distance_between_floor_endstops = 100 # Distance between endstops in cm (Check and adjust)!!!
        self.robot = Robot(self.distance_between_floor_endstops, status_log)
        
    def log(self, message, type = "info"):
        self.status_log.append({"source": "MainController", "message": message, "type": type, "time": datetime.now().isoformat()})

    def malowanie(self, x, z):
        self.log(f"Start malowania: X={x}cm, Z={z}cm", "info")
        step = 10  # wysokość kroku w cm
        current_z = z
        direction = True  # True = w prawo, False = w lewo

        # Ustaw platformę na wysokość początkową Z
        self.robot.platform.move_z_axis(current_z)
        self.log(f"Platforma ustawiona na wysokość {current_z} cm", "info")

        while current_z > 0:
            # Przejedź X w odpowiednim kierunku
            self.log(f"Jadę {'w prawo' if direction else 'w lewo'} na odległość {x} cm na wysokości {current_z} cm", "info")
            self.robot.move_forward(x if direction else -x, paint_or_not=True)
            self.log(f"Przejechano {x} cm na wysokości {current_z} cm", "success")

            # Zatrzymaj, opuść o 10 cm
            current_z -= step
            if current_z <= 0:
                self.log("Osiągnięto Z=0, koniec malowania.", "success")
                break
            self.robot.platform.move_z_axis(-step)
            self.log(f"Platforma opuszczona o {step} cm, nowa wysokość: {current_z} cm", "info")

            # Zmień kierunek jazdy
            direction = not direction

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
