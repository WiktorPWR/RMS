import RPi.GPIO as GPIO          # Import GPIO library for Raspberry Pi
from time import sleep, time     # Import sleep for delays and time for timestamps
import Components.constans
GPIO.setmode(GPIO.BCM)           # Use Broadcom (BCM) pin numbering
GPIO.setwarnings(False)          # Suppress GPIO warnings


class Motor():
    """
    Class to represent a motor in the robot. The motor is controlled using two GPIO pins for direction control
    and PWM for speed control.

    Attributes
    ----------
    pin_A : int
        The GPIO pin connected to the motor's A terminal.
    pin_B : int
        The GPIO pin connected to the motor's B terminal.

    Methods
    -------
    set_speed_motor(direction, speed)
        Sets the motor's speed and direction.
    """

    def __init__(self, pin_A, pin_B, status_log):
        """
        Initializes the motor with two GPIO pins for direction control.

        Parameters
        ----------
        pin_A : int
            The GPIO pin used for motor A.
        pin_B : int
            The GPIO pin used for motor B.
        """
        self.status_log = status_log
        self.pin_A = pin_A
        self.pin_B = pin_B

        GPIO.setup(pin_A, GPIO.OUT)
        GPIO.setup(pin_B, GPIO.OUT)

        self.pwm_1 = GPIO.PWM(pin_A, 100)  # Create PWM signal on pin A with 100Hz frequency
        self.pwm_2 = GPIO.PWM(pin_B, 100)  # Create PWM signal on pin B with 100Hz frequency

        self.pwm_1.start(0)  # Start with 0% duty cycle
        self.pwm_2.start(0)  # Start with 0% duty cycle

    def log(self, message, type="success"):
        self.status_log.append({"source": "[Motor]", "message": message, "type": type})

    def set_speed_motor(self, direction, speed):
        """
        Sets the direction and speed of the motor using PWM.

        Parameters
        ----------
        direction : bool
            True if the motor should move forward, False for backward.
        speed : int
            The desired speed (0-100), expressed as the duty cycle of the PWM.
        """
        if direction:
            # Move forward
            GPIO.output(self.pin_B, GPIO.LOW)
            GPIO.output(self.pin_A, GPIO.HIGH)
            self.pwm_1.ChangeDutyCycle(speed)
        else:
            # Move backward
            GPIO.output(self.pin_A, GPIO.LOW)
            GPIO.output(self.pin_B, GPIO.HIGH)
            self.pwm_2.ChangeDutyCycle(speed)
        self.log("MOve motor with speed {speed} ","info")

