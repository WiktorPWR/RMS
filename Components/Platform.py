import RPi.GPIO as GPIO          # Import GPIO library for Raspberry Pi
from time import sleep, time     # Import sleep for delays and time for timestamps
from Components.constans import SERVO_PIN,ENKODER_PIN_1,ENKODER_PIN_2
from Components.ScrewMotor import ScrewMotor
from Components.Endstop import Endstop
from Components.PaintSprayer import PaintSprayer
from Components.Ncoder import Ncoder
GPIO.setmode(GPIO.BCM)           # Use Broadcom (BCM) pin numbering
GPIO.setwarnings(False)          # Suppress GPIO warnings



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

    def __init__(self, pin_A, pin_B, endstop_up_pin, endstop_down_pin, status_log):
        """
        Initializes the platform with the motor
        
     
        """
        self.status_log = status_log
        self.screw_motor = ScrewMotor(pin_A, pin_B, status_log)
        self.endstop_up = Endstop(endstop_up_pin, status_log)
        self.endstop_down = Endstop(endstop_down_pin, status_log)
        self.paint_sprayer = PaintSprayer(SERVO_PIN, status_log)
        self.ncoder = Ncoder(ENKODER_PIN_1,ENKODER_PIN_2, status_log)

    def log(self, message, type="success"):
        self.status_log.append({"source": "[Platform]", "message": message, "type": type})

    def move_up(self, speed):
        """Move the platform upwards if the endstop is not triggered."""
        self.endstop_up.change_detected()
        if not self.endstop_up.change_detected():  # Ensure the platform has not reached the top
            self.screw_motor.move_up(speed)
     

    def move_down(self, speed):
        """Move the platform downwards if the endstop is not triggered."""
        self.endstop_down.change_detected()
        if not self.endstop_down.actual_state:  # Ensure the platform has not reached the bottom
            self.screw_motor.move_down(speed)

    def move_z_axis(self, distance):
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

        try:
            while current_position < target_distance:
                if direction_up:
                    if self.platform.endstop_max.change_detected():
                        self.platform.ncoder.set_position(self.platform.maks_height)
                        self.log(f"Endstop MAX triggered. Position set to {self.platform.maks_height} cm.", "warning")
                        break
                else:
                    if self.platform.endstop_min.change_detected():
                        self.platform.ncoder.set_position(0)
                        self.log("Endstop MIN triggered. Position set to 0 cm.", "warning")
                        break

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
                    if self.endstop_up.change_detected():
                        self.log(f"Endstop up detected", "warning") 
                        break
                    self.move_up(speed)
                else:
                    self.endstop_down.change_detected()
                    if self.endstop_down.actual_state:
                        self.log(f"Endstop down detected", "warning") 
                        break
                    self.move_down(speed)

                # Estymacja ruchu
                current_position = self.ncoder.update_position()

                self.log(f"Pos: {current_position:.2f}/{target_distance:.2f} cm | Spd: {speed}", "info")

        except KeyboardInterrupt:
            print(" Z-axis movement interrupted.")

        finally:
            self.stop()
            self.log("Target Z distance reached.","succes")


                

    def stop(self):
        """Stop the platform's movement."""
        self.screw_motor.stop()
        print("Platform stopped.")
