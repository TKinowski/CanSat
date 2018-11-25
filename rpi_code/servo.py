"""this file provides the <<servo>> class, which controlls a single servomotor"""


import RPi.GPIO as GPIO


class servo:
    def __init__(self, pin):
        """sets up the pwm output"""
        GPIO.setup(pin, GPIO.OUT)
        self.pwm = GPIO.PWM(pin, 50)
        self.pwm.start(0)
        #self.rotate_to(startpos)

    def rotate_to(self, position):
        position = float(position)
        """rotates the servo to a given position"""
        self.pwm.ChangeDutyCycle(2.3 + (position/180)*9)
    
    def stop(self):
        """sets duty cycle to zero to stop servo from shaking"""
        self.pwm.ChangeDutyCycle(0)