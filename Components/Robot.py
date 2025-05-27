import RPi.GPIO as GPIO          # Import GPIO library for Raspberry Pi
from constans import M1A,M1B,M2A,M2B,ECHO_PIN,TRIG_PIN,ENDSTOP1_PIN,ENDSTOP2_PIN,ENKODER_PIN_1,ENKODER_PIN_2,MZ1,MZ2,ENDSTOP3_PIN_MIN,ENDSTOP4_PIN_MAKS
import Endstop
import Motor
import Ncoder
import PaintSprayer
import Platform
import ScrewMotor
import Ultrasonic_sensor
from time import sleep, time     # Import sleep for delays and time for timestamps

GPIO.setmode(GPIO.BCM)           # Use Broadcom (BCM) pin numbering
GPIO.setwarnings(False)          # Suppress GPIO warnings

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

    def __init__(self,status_log,distance_between_floor_endstops):
        """
        Initializes the robot with motors and ultrasonic sensor.
        """
        self.status_log = status_log
        self.Motor_Left = Motor(M1A, M1B)
        self.Motor_Right = Motor(M2A, M2B)
        self.ultrasonik_sensor = Ultrasonic_sensor(ECHO_PIN, TRIG_PIN)
        self.ncoder_floor = Ncoder(ENKODER_PIN_1, ENKODER_PIN_2)
        self.endstop_floor_1 = Endstop(ENDSTOP1_PIN)
        self.endstop_floor_2 = Endstop(ENDSTOP2_PIN)
        self.platform = Platform(MZ1,MZ2,ENDSTOP3_PIN_MIN,ENDSTOP4_PIN_MAKS)
        self.maks_distance = distance_between_floor_endstops

    def log(self, message, type="success"):
        self.status_log.append({"source": "[Robot]", "message": message, "type": type})

    def move_forward(self, distance,paint_or_not=False):
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
        constans_speed = False
        self.ncoder_floor.reset_counter()
        current_position = 0.0
        self.is_moving = True

        try:
            while current_position <= target_position:

                if self.endstop_floor_1.change_detected():
                    current_position = 0
                    break
                elif self.endstop_floor_2.change_detected():
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

                if current_position >= acceleration_zone and current_position < remaining:
                    constans_speed = True
                else:
                    constans_speed = False
                
                if paint_or_not and constans_speed:
                    self.paint_sprayer.press()
                elif paint_or_not and not constans_speed:
                    self.paint_sprayer.release()
                # === 5. Apply to motors
                self.Motor_Left.set_speed_motor(direction, base_speed)
                self.Motor_Right.set_speed_motor(not direction, base_speed)

                # === 6. Update position estimate
              
                current_position = self.ncoder_floor.update_position()
                remaining = target_position - current_position

                self.status_log(f"Pos: {current_position:.2f} cm | Rem: {remaining:.2f} cm | Spd: {base_speed} | Moving: {self.is_moving}", "info")

                sleep(self.dt)

        except KeyboardInterrupt:
            print("[STOP] Interrupted by user")

        finally:
            self.Motor_Left.set_speed_motor(True, 0)
            self.Motor_Right.set_speed_motor(True, 0)
            self.is_moving = False
            self.status_log("Target distance reached.","success")

