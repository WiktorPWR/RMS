#This programe used to demonstare how to use Loch Antiphase with Hat-MDD10
#AN pin will act as sterring to control direction
#DIG pin will act to ON/OFF motor output.

import RPi.GPIO as GPIO			# using Rpi.GPIO module
from time import sleep			# import function sleep for delay
GPIO.setmode(GPIO.BCM)			# GPIO numbering
GPIO.setwarnings(False)			# enable warning from GPIO


M1A = 26
M1B = 19
M2A = 13
M2B = 6


class Ultrasonic_sensor():
    
    def __init__(self,pin_echo,pin_trig):
        self.pin_echo = pin_echo
        self.pin_trig = pin_trig
        
        GPIO.setup(pin_trig,GPIO.OUT)
        GPIO.setup(pin_echo,GPIO.IN)


    def get_distance():
        # Send a short pulse to the trigger pin
        GPIO.output(TRIG_PIN, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(TRIG_PIN, GPIO.LOW)

        # Measure the duration for the echo pulse
        while GPIO.input(ECHO_PIN) == 0:
            pulse_start = time.time()

        while GPIO.input(ECHO_PIN) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start

        # Calculate the distance based on the speed of sound (34300 cm/s)
        distance = pulse_duration * 34300 / 2

        return distance

    def filter_signal():
        filter_array = []
        num_samples = 20
        
        for i in range(num_samples):
            filter_array.append(get_distance())
            time.sleep(0.03)
        
        filter_array.sort()
        filtered_samples = filter_array[5:-5]
        
        distance = sum(filtered_samples) / len(filtered_samples)
        
        return distance





class Motor():
    """
    Class used to represent a motor in robot
    ...
    
    Attributes
    ----------
    pin_A: int
        Number of selected pin to driver
    pin_B: int
        Number of selected pin to driver
    
    Methodes
    ----------
    set_speed_motor(direction,speed)
        Function set a choosen speed on the motor
    
    
    """
    
    
    
    pin_A = 0
    pin_B = 0
    
    def __init__(self,pin_A,pin_B):
        self.pin_A = pin_A
        self.pin_B = pin_B
        
        GPIO.setup(pin_A,GPIO.OUT)
        GPIO.setup(pin_B,GPIO.OUT)
        
        pwm_1 = GPIO.PWM(pin_A,100)
        pwm_2 = GPIO.PWM(pin_B,100)
        
        pwm_1.start(0)
        pwm_2.start(0)
    
    def set_speed_motor(self,direction,speed):
        """
        Function set a choosen speed on the motor
        
        Parameters:
        ---------------------
        direction : bool
            1 - Left, 0 - Right
        
        speed : int
            Value of speed is set beetwen 0 and 100 % 
            0 - 0 rpm
            100 - 35 rpm
        
        """
        if(direction == 0):
            GPIO.setup(self.pin_A,GPIO.HIGH)
            pwm_1.ChangeDutyCycle(speed)
        else:
            GPIO.setup(self.pin_B,GPIO.HIGH)
            pwm_2.ChangeDutyCycle(speed)
    
    
class Robot():
    
                


while True:
    m1b.ChangeDutyCycle(10)
    GPIO.output(M1A,GPIO.LOW)
    m2b.ChangeDutyCycle(10)
    GPIO.output(M2A,GPIO.LOW)





































					
