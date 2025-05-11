# This program demonstrates how to control two DC motors using the MDD10 HAT in Lock-Antiphase mode.
# It uses an ultrasonic sensor to maintain a specified distance from a wall.
# The robot moves forward or backward while adjusting its speed based on proportional-integral control (PI).

import RPi.GPIO as GPIO          # Import GPIO library for Raspberry Pi
from time import sleep, time     # Import sleep for delays and time for timestamps

GPIO.setmode(GPIO.BCM)           # Use Broadcom (BCM) pin numbering
GPIO.setwarnings(False)          # Suppress GPIO warnings

# === GPIO Pin Definitions ===
TRIG_PIN = 10        # Trigger pin for ultrasonic sensor
ECHO_PIN = 11        # Echo pin for ultrasonic sensor
M1A = 26             # Motor 1 pin A
M1B = 19             # Motor 1 pin B
M2A = 13             # Motor 2 pin A
M2B = 6              # Motor 2 pin B
ENDSTOP1_PIN = 1     # Endstop start position 
ENDSTOP2_PIN = 2     # Endstop end position 
ENDSTOP3_PIN = 3     # Endstop Z homming


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
    actual_state = False

    def __init__(self, pin):

        self.endstop_pin = pin
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    def change_detected(self):
        """
        Detects the current state of the endstop

        Updates the 'actual_state' attribute depending on the GPIO input:
        - True if the endstop is triggered (logic LOW)
        - False if the endstop is not triggered (logic HIGH)
        """
        if not GPIO.input(self.endstop_pin):
            self.actual_state = True
        else:
            self.actual_state = False



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

    def __init__(self, pin_echo, pin_trig):
        """
        Initializes the ultrasonic sensor with the given ECHO and TRIG pin numbers.

        Parameters
        ----------
        pin_echo : int
            The GPIO pin used for receiving the echo signal.
        pin_trig : int
            The GPIO pin used for sending the trigger signal.
        """
        self.pin_echo = pin_echo
        self.pin_trig = pin_trig

        GPIO.setup(self.pin_trig, GPIO.OUT)
        GPIO.setup(self.pin_echo, GPIO.IN)

    def get_distance(self):
        """
        Perform a single measurement of the distance using the ultrasonic sensor.

        Returns
        -------
        float
            The distance to the object in centimeters.
        """
        # Send a short pulse to trigger the sensor
        GPIO.output(self.pin_trig, GPIO.HIGH)
        sleep(0.00001)
        GPIO.output(self.pin_trig, GPIO.LOW)

        # Measure pulse duration to calculate distance
        while GPIO.input(self.pin_echo) == 0:
            pulse_start = time()

        while GPIO.input(self.pin_echo) == 1:
            pulse_end = time()

        pulse_duration = pulse_end - pulse_start

        # Calculate distance (speed of sound = 34300 cm/s)
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
        return distance


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

    def __init__(self, pin_A, pin_B):
        """
        Initializes the motor with two GPIO pins for direction control.

        Parameters
        ----------
        pin_A : int
            The GPIO pin used for motor A.
        pin_B : int
            The GPIO pin used for motor B.
        """
        self.pin_A = pin_A
        self.pin_B = pin_B

        GPIO.setup(pin_A, GPIO.OUT)
        GPIO.setup(pin_B, GPIO.OUT)

        self.pwm_1 = GPIO.PWM(pin_A, 100)  # Create PWM signal on pin A with 100Hz frequency
        self.pwm_2 = GPIO.PWM(pin_B, 100)  # Create PWM signal on pin B with 100Hz frequency

        self.pwm_1.start(0)  # Start with 0% duty cycle
        self.pwm_2.start(0)  # Start with 0% duty cycle

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


class Ncoder():
    """
    Class used to simulate an encoder for the robot. The encoder tracks the robot's position.

    Attributes
    ----------
    Current_position : int
        Variable to hold the current position of the encoder.

    Methods
    -------
    update_position(value)
        Updates the encoder's position with a specific value.
    
    reset_cunter()
        Resets the encoder's position counter.
    """

    Current_position = 0

    def update_position(self, value):
        """
        Update the current position of the encoder.

        Parameters
        ----------
        value : int
            The new position value to update the encoder's position.

        Returns
        -------
        int
            The updated position.
        """
        return value

    def reset_cunter(self):
        """
        Resets the encoder's position to 0.
        """
        self.Current_position = 0


class Robot():
    """
    Class representing a robot with two motors and an ultrasonic sensor.

    Attributes
    ----------
    Motor_Left : Motor
        An instance of the left motor.
    Motor_Right : Motor
        An instance of the right motor.
    ultrasonik_sensor : Ultrasonic_sensor
        An instance of the ultrasonic sensor.
    target_distance_from_wall : float
        The target distance from the wall that the robot should maintain (in cm).
    Kp : float
        The proportional gain for the control system.
    Ki : float
        The integral gain for the control system.
    max_speed : int
        The maximum speed the motors can run at (0-100).
    dt : float
        The time step for the control loop (in seconds).

    Methods
    -------
    move_forward(distance)
        Moves the robot forward or backward while maintaining a specified distance from the wall using PI control.
    """

    Kp = 4                     # Proportional gain
    Ki = 0.5                   # Integral gain
    max_speed = 100            # Maximum speed (PWM duty cycle)
    dt = 0.1                   # Control loop time step (s)
    target_distance_from_wall = 10  # Desired distance from wall in cm

    def __init__(self):
        """
        Initializes the robot with motors and ultrasonic sensor.
        """
        self.Motor_Left = Motor(M1A, M1B)
        self.Motor_Right = Motor(M2A, M2B)
        self.ultrasonik_sensor = Ultrasonic_sensor(ECHO_PIN, TRIG_PIN)
        self.ncoder_floor = Ncoder()
        self.endstop_floor_1 = Endstop(ENDSTOP1_PIN)
        self.endstop_floor_2 = Endstop(ENDSTOP2_PIN)
        self.endstop_Z_axis = Endstop(ENDSTOP3_PIN)

    def move_forward(self, distance):
        """
        Moves the robot forward or backward while adjusting its speed to maintain a target distance from the wall.
        The movement is controlled using a proportional-integral control (PI) system.

        Parameters
        ----------
        distance : float
            The target distance (positive for forward, negative for backward).
        """
        direction = True if distance >= 0 else False
        error_integral = 0.0

        try:
            while True:
                # 1. Measure current distance from the wall
                current_distance = self.ultrasonik_sensor.filter_signal()

                # 2. Calculate the error (difference between target and current distance)
                error = self.target_distance_from_wall - current_distance

                # 3. Update integral term (with anti-windup)
                error_integral += error * self.dt
                error_integral = max(-50, min(50, error_integral))  # Clamping integral value

                # 4. Compute the control output
                control_output = self.Kp * error + self.Ki * error_integral

                # 5. Limit the speed to the maximum allowed speed
                speed = min(abs(int(control_output)), self.max_speed)

                # 6. Drive the motors (opposite directions for differential drive)
                self.Motor_Left.set_speed_motor(direction, speed)
                self.Motor_Right.set_speed_motor(not direction, speed)

                # 7. Print debug information
                print(f"[INFO] Dystans: {current_distance:.2f} cm | Błąd: {error:.2f} | Prędkość: {speed}")

                sleep(self.dt)

        except KeyboardInterrupt:
            # Stop motors when interrupted
            self.Motor_Left.set_speed_motor(True, 0)
            self.Motor_Right.set_speed_motor(True, 0)
            GPIO.cleanup()
            print("[STOP] Program zatrzymany.")


# Main setup

robot = Robot()
GPIO.add_event_detect(robot.endstop_floor_1.endstop_pin,GPIO.BOTH,callback=robot.endstop_floor_1.change_detected,bouncetime=robot.endstop_floor_1.bounce_time)
GPIO.add_event_detect(robot.endstop_floor_2.endstop_pin,GPIO.BOTH,callback=robot.endstop_floor_1.change_detected,bouncetime=robot.endstop_floor_2.bounce_time)

# === Main Test Sequence ===


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
