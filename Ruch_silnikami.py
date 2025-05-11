# This program is used to demonstrate how to use Lock Antiphase with Hat-MDD10
# AN pin will act as steering to control direction
# DIG pin will act to ON/OFF motor output.

import RPi.GPIO as GPIO          # using RPi.GPIO module
from time import sleep, time     # import sleep for delay and time for timestamping

GPIO.setmode(GPIO.BCM)           # Use BCM GPIO numbering
GPIO.setwarnings(False)          # Disable GPIO warnings

# Define GPIO pins
TRIG_PIN = 10
ECHO_PIN = 11
M1A = 26
M1B = 19
M2A = 13
M2B = 6


class Ultrasonic_sensor():
    """
    Class used to handle ultrasonic distance measurement
    ...
    
    Attributes
    ----------
    pin_echo : int
        Pin number connected to ECHO signal
    pin_trig : int
        Pin number connected to TRIG signal
    
    Methods
    -------
    get_distance()
        Returns a single distance measurement in centimeters
    
    filter_signal()
        Returns filtered average of multiple distance samples
    """

    def __init__(self, pin_echo, pin_trig):
        self.pin_echo = pin_echo
        self.pin_trig = pin_trig

        GPIO.setup(self.pin_trig, GPIO.OUT)
        GPIO.setup(self.pin_echo, GPIO.IN)

    def get_distance(self):
        """
        Function to perform a single measurement using the ultrasonic sensor.
        
        Returns:
        --------
        distance : float
            Distance from object in cm
        """
        # Send a short pulse to the trigger pin
        GPIO.output(self.pin_trig, GPIO.HIGH)
        sleep(0.00001)
        GPIO.output(self.pin_trig, GPIO.LOW)

        # Measure the duration of the echo pulse
        while GPIO.input(self.pin_echo) == 0:
            pulse_start = time()

        while GPIO.input(self.pin_echo) == 1:
            pulse_end = time()

        pulse_duration = pulse_end - pulse_start

        # Calculate the distance based on the speed of sound (34300 cm/s)
        distance = pulse_duration * 34300 / 2

        return distance

    def filter_signal(self):
        """
        Perform multiple measurements and average a filtered subset to reduce noise
        
        Returns:
        --------
        distance : float
            Filtered average distance
        """
        filter_array = []
        num_samples = 20

        for i in range(num_samples):
            filter_array.append(self.get_distance())
            sleep(0.03)

        filter_array.sort()
        filtered_samples = filter_array[5:-5]  # Remove extreme values

        distance = sum(filtered_samples) / len(filtered_samples)
        return distance


class Motor():
    """
    Class used to represent a motor in robot
    ...
    
    Attributes
    ----------
    pin_A : int
        Number of selected GPIO pin for motor control
    pin_B : int
        Number of selected GPIO pin for motor control
    
    Methods
    -------
    set_speed_motor(direction, speed)
        Sets desired direction and speed to the motor
    """

    def __init__(self, pin_A, pin_B):
        self.pin_A = pin_A
        self.pin_B = pin_B

        GPIO.setup(pin_A, GPIO.OUT)
        GPIO.setup(pin_B, GPIO.OUT)

        self.pwm_1 = GPIO.PWM(pin_A, 100)  # Create PWM channel on pin A with 100Hz
        self.pwm_2 = GPIO.PWM(pin_B, 100)  # Create PWM channel on pin B with 100Hz

        self.pwm_1.start(0)
        self.pwm_2.start(0)

    def set_speed_motor(self, direction, speed):
        """
        Function to set direction and speed of the motor

        Parameters:
        -----------
        direction : bool
            False - backward, True - forward
        
        speed : int
            Value of speed set between 0 and 100 %
        """
        if direction:
            # Forward
            GPIO.output(self.pin_B, GPIO.LOW)
            GPIO.output(self.pin_A, GPIO.HIGH)
            self.pwm_1.ChangeDutyCycle(speed)
        else:
            # Backward
            GPIO.output(self.pin_A, GPIO.LOW)
            GPIO.output(self.pin_B, GPIO.HIGH)
            self.pwm_2.ChangeDutyCycle(speed)


class Ncoder():
    """
    Class used to represent nkoder of robot
    ...

    Attributes
    -----------
    Current_position : int
        Varaible to hold current position of encoder

    Methods
    -----------
    update_position(value)
        Variable Current_position is change by specific value 

    reset_cunter()
        Function to reset counting position

    """
    Current_position = 0

    def update_position(self,value):
        # Nastepuje aktualizcja pozycji
        return value 

    def reset_cunter(self):
        # Zresetowanie licznika
        self.Current_position = 0
         


class Robot():
    """
    Class used to encapsulate full robot system with two motors and one ultrasonic sensor
    ...
    
    Attributes
    ----------
    Motor_Left : Motor
        Instance controlling left motor
    Motor_Right : Motor
        Instance controlling right motor
    ultrasonik_sensor : Ultrasonic_sensor
        Instance of ultrasonic sensor
    Distance_from_wall : float
        Distance in centimeters from wall that should be set all the time
    
    Methods
    -------
    (none yet - extendable)
    """

    def __init__(self):
        self.Motor_Left = Motor(M1A, M1B)
        self.Motor_Right = Motor(M2A, M2B)
        self.ultrasonik_sensor = Ultrasonic_sensor(ECHO_PIN, TRIG_PIN)
        self.ncoder_floor = Ncoder()

    def move_robot(self,distance):



robot = Robot()

try:
    while True:
        print("Przód")
        robot.Motor_Left.set_speed_motor(True, 70)
        robot.Motor_Right.set_speed_motor(True, 70)
        sleep(2)

        print("Stop")
        robot.Motor_Left.set_speed_motor(True, 0)
        robot.Motor_Right.set_speed_motor(True, 0)
        sleep(1)

        print("Tył")
        robot.Motor_Left.set_speed_motor(False, 70)
        robot.Motor_Right.set_speed_motor(False, 70)
        sleep(2)

        print("Stop")
        robot.Motor_Left.set_speed_motor(False, 0)
        robot.Motor_Right.set_speed_motor(False, 0)
        sleep(3)

except KeyboardInterrupt:
    GPIO.cleanup()
    print("Program zatrzymany.")
