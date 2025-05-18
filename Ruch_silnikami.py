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
SERVO_PIN = 20
ENKODER_PIN_1 = 4
ENKODER_PIN_2 = 5

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

    def __init__(self, pin_a, pin_b=None):
        """
        Initializes the encoder using the given GPIO pins

        Parameters
        ----------
        pin_a : int
            GPIO pin number for encoder channel A
        pin_b : int or None
            GPIO pin number for encoder channel B (optional)
        """
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.Current_position = 0
        self.distance_per_tick = 0.1  # [NOTE] Adjust this value based on real-world testing

        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        if self.pin_b:
            GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # [INFO] Set up interrupt on both rising and falling edges for pin A
        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self.encoder_callback, bouncetime=1)

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

    def __init__(self, servo_pin):
        self.servo_pin = servo_pin
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.servo_pin, 50)  
        self.pwm.start(0)
        sleep(0.5)
        self.release()

    def press(self):
        """naciskanie puchy"""
        self.pwm.ChangeDutyCycle(7.5)  # to jest 90 stopni, dostosuje sie juz przy testach
        sleep(0.5)
        self.pwm.ChangeDutyCycle(0)  

    def release(self):
        """serwo w pozycji zero(poczatkowej)"""
        self.pwm.ChangeDutyCycle(2.5)  # kat zero stopni
        sleep(0.5)
        self.pwm.ChangeDutyCycle(0)

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

    def __init__(self, pin_A, pin_B):
       
        self.pin_A = pin_A
        self.pin_B = pin_B

        GPIO.setup(pin_A, GPIO.OUT)
        GPIO.setup(pin_B, GPIO.OUT)

        # Create PWM signal on pin A with 100Hz frequency
        self.pwm_1 = GPIO.PWM(pin_A, 100)
        self.pwm_2 = GPIO.PWM(pin_B, 100)

        self.pwm_1.start(0) 
        self.pwm_2.start(0)  

    def move_up(self, speed):
        """
       platforma do gory
       predkosc (0-100), PWM
        """
        GPIO.output(self.pin_B, GPIO.LOW)
        GPIO.output(self.pin_A, GPIO.HIGH)
        self.pwm_1.ChangeDutyCycle(speed)

    def move_down(self, speed):
        """
        platforma w dół

        """
        GPIO.output(self.pin_A, GPIO.LOW)
        GPIO.output(self.pin_B, GPIO.HIGH)
        self.pwm_2.ChangeDutyCycle(speed)

    def stop(self):
        """zatrzymuje platforme"""

        self.pwm_1.ChangeDutyCycle(0)
        self.pwm_2.ChangeDutyCycle(0)

class Platform():
    """
    Class to control the movement of the platform on the trapezoidal screw.
    
    Attributes
    ----------
    screw_motor : ScrewMotor
        The screw motor controlling the platform's movement.
    endstop_up : Endstop
        The endstop for the upward limit.
    endstop_down : Endstop
        The endstop for the downward limit.
    """

    def __init__(self, pin_A, pin_B, endstop_up_pin, endstop_down_pin):
        """
        Initializes the platform with the motor
        
     
        """
        self.screw_motor = ScrewMotor(pin_A, pin_B)
        self.endstop_up = Endstop(endstop_up_pin)
        self.endstop_down = Endstop(endstop_down_pin)
        self.paint_sprayer = PaintSprayer(SERVO_PIN)
        self.ncoder = Ncoder(ENKODER_PIN_1,ENKODER_PIN_2)

    def move_up(self, speed):
        """Move the platform upwards if the endstop is not triggered."""
        self.endstop_up.change_detected()
        if not self.endstop_up.actual_state:  # Ensure the platform has not reached the top
            self.screw_motor.move_up(speed)
     

    def move_down(self, speed):
        """Move the platform downwards if the endstop is not triggered."""
        self.endstop_down.change_detected()
        if not self.endstop_down.actual_state:  # Ensure the platform has not reached the bottom
            self.screw_motor.move_down(speed)

    def move_z_axis(self, distance, paint_or_not=False):
        """
        Moves the platform up or down by a specified distance, with acceleration/deceleration.
        Optionally activates the paint sprayer for the entire movement.

        Parameters
        ----------
        distance : float
            Distance to move the platform in cm (positive = up, negative = down).
        paint_or_not : bool
            If True, activates spray during entire movement.
        dt : float
            Delay time for loop cycle in seconds.
        servo_pin : int or None
            GPIO pin connected to the spray servo (required if paint_or_not=True).
        """

        direction_up = distance > 0
        target_distance = abs(distance)
        current_position = 0.0
        max_speed = 80     # Max PWM
        min_speed = 30     # Minimum effective PWM
        acceleration_zone = 3.0  # cm

        if paint_or_not:
            self.paint_sprayer.press()
            print("[SPRAY] Spray activated.")

        try:
            while current_position < target_distance:
                remaining = target_distance - current_position

                # Zwalnianie przy końcu
                if remaining < 0.5:
                    speed = 0
                elif remaining < 1.0:
                    speed = int(max_speed * 0.25)
                elif remaining < 2.0:
                    speed = int(max_speed * 0.5)
                elif remaining < acceleration_zone:
                    speed = int(max_speed * 0.75)
                elif current_position < acceleration_zone:
                    speed = int(max_speed * (current_position / acceleration_zone))
                else:
                    speed = max_speed

                speed = max(min_speed, speed)

                if direction_up:
                    self.endstop_up.change_detected()
                    if self.endstop_up.actual_state:
                        print("[STOP] Endstop UP triggered")
                        break
                    self.move_up(speed)
                else:
                    self.endstop_down.change_detected()
                    if self.endstop_down.actual_state:
                        print("[STOP] Endstop DOWN triggered")
                        break
                    self.move_down(speed)

                # Estymacja ruchu
                current_position = self.ncoder.update_position()

                print(f"[Z-MOVE] Pos: {current_position:.2f}/{target_distance:.2f} cm | Spd: {speed}")

        except KeyboardInterrupt:
            print("[INTERRUPT] Z-axis movement interrupted.")

        finally:
            self.stop()
            print("[DONE] Target Z distance reached.")
            if self.paint_sprayer:
                self.paint_sprayer.release()
                print("[SPRAY] Spray released.")

                

    def stop(self):
        """Stop the platform's movement."""
        self.screw_motor.stop()
        print("Platform stopped.")


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
        sleep(0.0001)
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
    maks_distance = 0

    def __init__(self,distance_between_floor_endstops):
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
        self.maks_distance = distance_between_floor_endstops

    def move_forward(self, distance):
        """
        Moves the robot forward or backward while adjusting its speed to maintain a target distance from the wall.
        Includes smooth acceleration and deceleration. The movement is controlled using a PI system.
        Stops automatically after traveling the specified distance.

        Parameters
        ----------
        distance : float
            The target distance (positive for forward, negative for backward).
        is_moving : bool
            Flag indicating whether the robot is currently moving (True) or stationary (False).

        """
        direction = True if distance >= 0 else False
        error_integral = 0.0
        target_position = abs(distance)  # target travel distance in cm

        self.ncoder_floor.reset_counter()
        current_position = 0.0
        self.is_moving = True

        try:
            while current_position <= target_position:

                if(self.endstop_floor_1 == True):
                    current_position = 0
                    break
                elif (self.endstop_floor_2 == True):
                    current_position = self.maks_distance
                    break

                # === 1. Measure current distance from the wall
                current_distance = self.ultrasonik_sensor.filter_signal()

                # === 2. PI controller for wall distance
                error = self.target_distance_from_wall - current_distance
                error_integral += error * self.dt
                error_integral = max(-50, min(50, error_integral))  # antywindup

                base_output = self.Kp * error + self.Ki * error_integral
                base_speed = min(abs(int(base_output)), self.max_speed)


                # === 3. Smooth acceleration
                acceleration_zone = 3.0  # cm for acceleration
                if current_position < acceleration_zone:
                    acceleration_factor = current_position / acceleration_zone
                    base_speed = int(base_speed * acceleration_factor)

                # === 4. Smooth deceleration
                remaining = target_position - current_position
                if remaining < 3.0:
                    if remaining < 0.5:
                        base_speed = 0
                    elif remaining < 1.0:
                        base_speed = int(base_speed * 0.25)
                    elif remaining < 2.0:
                        base_speed = int(base_speed * 0.5)
                    else:
                        base_speed = int(base_speed * 0.75)

                # === 5. Apply to motors
                self.Motor_Left.set_speed_motor(direction, base_speed)
                self.Motor_Right.set_speed_motor(not direction, base_speed)

                # === 6. Update position estimate
                distance_per_loop = base_speed * self.dt * 0.1  # adjust this factor to your robot
                current_position = self.ncoder_floor.update_position(distance_per_loop)
                remaining = target_position - current_distance

                print(f"[INFO] Pos: {current_position:.2f} cm | Rem: {remaining:.2f} cm | Spd: {base_speed} | Moving: {self.is_moving}")

                sleep(self.dt)

        except KeyboardInterrupt:
            print("[STOP] Interrupted by user")

        finally:
            self.Motor_Left.set_speed_motor(True, 0)
            self.Motor_Right.set_speed_motor(True, 0)
            self.is_moving = False
            print("[DONE] Target distance reached.")



# Main setup

robot = Robot()

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
