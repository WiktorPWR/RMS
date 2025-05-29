import RPi.GPIO as GPIO          # Import GPIO library for Raspberry Pi
from time import sleep, time     # Import sleep for delays and time for timestamps
import Components.constans
GPIO.setmode(GPIO.BCM)           # Use Broadcom (BCM) pin numbering
GPIO.setwarnings(False)          # Suppress GPIO warnings



class PaintSprayer():
    """
    Class to handle a servo that presses the spray can in the wall painting robot.
    
    Attributes
    ----------
    servo_pin : int
        GPIO pin connected to the servo control wire.
    pwm : GPIO.PWM
        PWM object to control the servo.
    
    Methods
    -------
    press()
        Presses the spray can.
    release()
        Releases the spray can.
   
    """

    def __init__(self, servo_pin, status_log):
        self.status_log = status_log
        self.servo_pin = servo_pin
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.servo_pin, 50)  
        self.pwm.start(0)
        sleep(0.5)
        self.release()

    def log(self, message, type="success"):
        self.status_log.append({"source": "[PaintSprayer]", "message": message, "type": type})

    def press(self):
        """naciskanie puchy"""
        self.log(f"Naciskanie puchy", "info")
        self.pwm.ChangeDutyCycle(7.5)  # to jest 90 stopni, dostosuje sie juz przy testach
        sleep(0.5)
        self.pwm.ChangeDutyCycle(0)  


    def release(self):
        """serwo w pozycji zero(poczatkowej)"""
        self.log(f"Puszczenie puchy","info")
        self.pwm.ChangeDutyCycle(2.5)  # kat zero stopni
        sleep(0.5)
        self.pwm.ChangeDutyCycle(0)
