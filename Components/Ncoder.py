import RPi.GPIO as GPIO          # Import GPIO library for Raspberry Pi
import Components.constans
from time import sleep, time     # Import sleep for delays and time for timestamps
from datetime import datetime

GPIO.setmode(GPIO.BCM)           # Use Broadcom (BCM) pin numbering
GPIO.setwarnings(False)          # Suppress GPIO warnings

class Ncoder():
    """
    Class for handling an incremental rotary encoder (e.g. KY-040)

    ...

    Attributes
    ----------
    pin_a : int
        GPIO pin connected to encoder channel A (required)
    pin_b : int or None
        GPIO pin connected to encoder channel B (optional, used for direction detection)
    Current_position : int
        Current number of counted encoder pulses
    distance_per_tick : float
        Conversion factor from encoder ticks to distance in centimeters

    Methods
    -------
    encoder_callback(channel)
        Function called on signal change on encoder pin A (interrupt)
    update_position()
        Returns the current distance in centimeters
    reset_counter()
        Resets the encoder tick counter to zero
    """

    def __init__(self, status_log, pin_a, pin_b=None):
        """
        Initializes the encoder using the given GPIO pins

        Parameters
        ----------
        pin_a : int
            GPIO pin number for encoder channel A
        pin_b : int or None
            GPIO pin number for encoder channel B (optional)
        """
        self.status_log = status_log
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.Current_position = 0
        self.distance_per_tick = 0.1  # [NOTE] Adjust this value based on real-world testing

        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        if self.pin_b:
            GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # [INFO] Set up interrupt on both rising and falling edges for pin A
        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self.encoder_callback, bouncetime=1)

    def log(self, message, type="success"):
        self.status_log.append({"source": "Ncoder", "message": message, "type": type, "time": datetime.now().isoformat()})

    def encoder_callback(self, channel):
        """
        Interrupt callback for encoder pulse counting

        If channel B is defined, it is used to determine rotation direction.
        Otherwise, each pulse is counted as forward movement.
        """
        
        if self.pin_b:
            # [INFO] Direction detection: HIGH = forward, LOW = reverse
            if GPIO.input(self.pin_b) == GPIO.HIGH:
                self.Current_position += 1
            else:
                self.Current_position -= 1
        else:
            # [INFO] No direction detection — assuming forward movement only
            self.Current_position += 1
        
        self.log(f"Actual position: {self.Current_position} cm", "info")  

    def update_position(self):
        """
        Returns the current estimated distance based on encoder pulses

        Returns
        -------
        float
            Distance in centimeters
        """
        return self.Current_position * self.distance_per_tick

    def reset_counter(self):
        """
        Resets the encoder pulse counter to zero

        This should be used when reaching a known physical reference point,
        such as a limit switch (endstop).
        """
        # [INFO] Reset encoder position to 0 cm
        self.Current_position = 0
        self.log(f"Reset counter", "info")
