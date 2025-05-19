import RPi.GPIO as GPIO          # Import GPIO library for Raspberry Pi
from time import sleep, time     # Import sleep for delays and time for timestamps
import constans
GPIO.setmode(GPIO.BCM)           # Use Broadcom (BCM) pin numbering
GPIO.setwarnings(False)          # Suppress GPIO warnings



class Ultrasonic_sensor():
    """
    Class used to handle ultrasonic distance measurement.
    It uses the HC-SR04 ultrasonic sensor to measure the distance to an object.

    Attributes
    ----------
    pin_echo : int
        Pin number connected to ECHO signal.
    pin_trig : int
        Pin number connected to TRIG signal.

    Methods
    -------
    get_distance()
        Performs a single distance measurement and returns the distance in centimeters.
    
    filter_signal()
        Performs multiple measurements and returns the filtered average distance to reduce noise.
    """

    def __init__(self,status_log, pin_echo, pin_trig):
        """
        Initializes the ultrasonic sensor with the given ECHO and TRIG pin numbers.

        Parameters
        ----------
        pin_echo : int
            The GPIO pin used for receiving the echo signal.
        pin_trig : int
            The GPIO pin used for sending the trigger signal.
        """
        self.status_log = status_log
        self.pin_trig = pin_trig
        self.pin_echo = pin_echo
        GPIO.setup(pin_trig, GPIO.OUT)
        GPIO.setup(pin_echo, GPIO.IN)
        GPIO.output(pin_trig, GPIO.LOW)

    def log(self, message, type="success"):
        self.status_log.append({"source": "[Ultrasonic_sensor]", "message": message, "type": type})

    def get_distance(self):
        """
        Perform a single measurement of the distance using the ultrasonic sensor.

        Returns
        -------
        float
            The distance to the object in centimeters.
        """
        GPIO.output(self.pin_trig, GPIO.HIGH)
        sleep(0.0001)
        GPIO.output(self.pin_trig, GPIO.LOW)

        timeout_start = time() + 0.1
        while GPIO.input(self.pin_echo) == 0:
            pulse_start = time()
            if pulse_start > timeout_start:
                return -1

        timeout_end = time() + 0.1
        while GPIO.input(self.pin_echo) == 1:
            pulse_end = time()
            if pulse_end > timeout_end:
                return -1

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 34300 / 2
        return distance
    
    def filter_signal(self):
        """
        Collects multiple measurements to reduce noise by averaging the values.
        
        Returns
        -------
        float
            The filtered average distance in centimeters.
        """
        filter_array = []
        num_samples = 20  # Number of samples to collect

        for i in range(num_samples):
            filter_array.append(self.get_distance())
            sleep(0.03)  # Small delay between samples

        # Sort the data and remove extreme values
        filter_array.sort()
        filtered_samples = filter_array[5:-5]  # Remove the top and bottom 5 samples

        # Return the average of the filtered samples
        distance = sum(filtered_samples) / len(filtered_samples)
        self.log("Aktual distance from the wall: {distance}","info")
        return distance
