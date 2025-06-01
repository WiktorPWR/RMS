import RPi.GPIO as GPIO          # Import GPIO library for Raspberry Pi
from time import sleep, time     # Import sleep for delays and time for timestamps
import Components.constans
GPIO.setmode(GPIO.BCM)           # Use Broadcom (BCM) pin numbering
GPIO.setwarnings(False)          # Suppress GPIO warnings


class ScrewMotor():
    """
    Class to handle the control of the trapezoidal screw motor.
    
    Attributes
    ----------
    pin_A : int
        The GPIO pin for the motor A.
    pin_B : int
        The GPIO pin for the motor B.
    pwm : PWM instance
        The PWM instance controlling the motor's speed.

    Methods
    -------
    move_up(speed)
        Moves the platform upwards at the specified speed.
    move_down(speed)
        Moves the platform downwards at the specified speed.
    stop()
        Stops the motor.
    """

    def __init__(self, pin_A, pin_B, status_log):
        self.status_log = status_log
        self.pin_A = pin_A
        self.pin_B = pin_B

        GPIO.setup(pin_A, GPIO.OUT)
        GPIO.setup(pin_B, GPIO.OUT)

        # Create PWM signal on pin A with 100Hz frequency
        self.pwm_1 = GPIO.PWM(pin_A, 100)
        self.pwm_2 = GPIO.PWM(pin_B, 100)

        self.pwm_1.start(0) 
        self.pwm_2.start(0)  

    def log(self, message, type="success"):
        self.status_log.append({"source": "[ScrewMotor]", "message": message, "type": type})

    def move_up(self, speed):
        """
       platforma do gory
       predkosc (0-100), PWM
        """
        self.log(f"We go down with speed {speed}", "info")
        GPIO.output(self.pin_B, GPIO.LOW)
        GPIO.output(self.pin_A, GPIO.HIGH)
        self.pwm_1.ChangeDutyCycle(speed)

    def move_down(self, speed):
        """
        platforma w dół

        """
        self.log(f"We go up with speed {speed}", "info")
        GPIO.output(self.pin_A, GPIO.LOW)
        GPIO.output(self.pin_B, GPIO.HIGH)
        self.pwm_2.ChangeDutyCycle(speed)

    def stop(self):
        """zatrzymuje platforme"""
        self.log(f"We STOP", "warning")
        self.pwm_1.ChangeDutyCycle(0)
        self.pwm_2.ChangeDutyCycle(0)
