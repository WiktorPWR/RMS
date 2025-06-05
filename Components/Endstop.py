import RPi.GPIO as GPIO          # Import GPIO library for Raspberry Pi
from time import sleep, time     # Import sleep for delays and time for timestamps
import Components.constans
from datetime import datetime
GPIO.setmode(GPIO.BCM)           # Use Broadcom (BCM) pin numbering
GPIO.setwarnings(False)          # Suppress GPIO warnings


class Endstop():
    """
    Class representing a limit switch (endstop)
    ...
    
    Attributes
    ----------

    bounce_time : int 
        Time in milisecods in which CPU ignores other trigeers like that it is mechanizm to eliminate bouncing

    actual_state : bool
        Current state of the endstop (True = triggered, False = not triggered)

    endstop_pin : int
        GPIO pin number connected to the endstop

    Methods
    -------
    change_detected()
        Checks the GPIO pin and updates the actual state of the endstop
    """
    bounce_time = 50
    def __init__(self, pin, status_log):
        self.status_log = status_log
        self.endstop_pin = pin
        self.actual_state = False
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    def log(self, message, type="success"):
        self.status_log.append({"source": "Endstop", "message": message, "type": type, "time": datetime.now().isoformat()})

    def change_detected(self):
        """
        Detects the current state of the endstop

        Updates the 'actual_state' attribute depending on the GPIO input:
        - True if the endstop is triggered (logic LOW)
        - False if the endstop is not triggered (logic HIGH)
        """
        self.log("Endstop change state","warning")
        return GPIO.input(self.endstop_pin) == self.actual_state
